use super::*;
use crate::{circle_angle_to_parameter, KNOT_IDENTITY_TOL};

/// Axis/radius description shared by the cylinder and cone wall builders:
/// `station(p)` is the axial coordinate, `radius_at(s)` the wall radius.
struct RevolveInfo {
    origin: Vec3,
    axis: Vec3,
    sense: i8,
    radius_slope: f64,
    radius_offset: f64,
}

impl RevolveInfo {
    fn from_carrier(carrier: &RegionCarrier) -> Option<RevolveInfo> {
        match carrier {
            RegionCarrier::Cylinder {
                axis_point,
                axis_dir,
                radius,
                sense,
            } => Some(RevolveInfo {
                origin: *axis_point,
                axis: *axis_dir,
                sense: *sense,
                radius_slope: 0.0,
                radius_offset: *radius,
            }),
            RegionCarrier::Cone {
                apex,
                axis_dir,
                half_angle_rad,
                sense,
            } => Some(RevolveInfo {
                origin: *apex,
                axis: *axis_dir,
                sense: *sense,
                radius_slope: half_angle_rad.tan(),
                radius_offset: 0.0,
            }),
            _ => None,
        }
    }

    fn station(&self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.axis)
    }

    fn radius_at(&self, station: f64) -> f64 {
        self.radius_offset + self.radius_slope * station
    }

    fn axis_point_at(&self, station: f64) -> Vec3 {
        self.origin.add(self.axis.scale(station))
    }

    fn radial(&self, point: Vec3) -> Vec3 {
        let d = point.sub(self.origin);
        d.sub(self.axis.scale(d.dot(self.axis)))
    }
}

/// Statistics of a chain against a revolve carrier: mean axial station and
/// worst deviations from "axis-perpendicular circle at that station".
struct RingFit {
    station: f64,
    station_spread: f64,
    radial_spread: f64,
}

fn ring_fit(builder: &RegionBrepBuilder, info: &RevolveInfo, chain: &Chain) -> RingFit {
    let points = builder.chain_points(chain);
    let mean = points.iter().map(|&p| info.station(p)).sum::<f64>() / points.len() as f64;
    let mut station_spread = 0.0_f64;
    let mut radial_spread = 0.0_f64;
    let radius = info.radius_at(mean);
    for &p in &points {
        station_spread = station_spread.max((info.station(p) - mean).abs());
        radial_spread = radial_spread.max((info.radial(p).length() - radius).abs());
    }
    RingFit {
        station: mean,
        station_spread,
        radial_spread,
    }
}

/// Dispatch a cylinder/cone region to the full-wall or partial-patch
/// builder based on its boundary structure.
pub(super) fn build_revolved_region_face(
    builder: &mut RegionBrepBuilder,
    region: &MeshRegion,
    cycles: &[Vec<usize>],
) -> Result<FaceRecord, String> {
    let info = RevolveInfo::from_carrier(&region.carrier)
        .expect("caller dispatches only revolve carriers");
    let traversals = cycles
        .iter()
        .map(|cycle| builder.cycle_traversals(cycle))
        .collect::<Result<Vec<_>, _>>()?;
    let all_rings = traversals
        .iter()
        .all(|t| t.len() == 1 && builder.chains[t[0].chain].closed);
    if cycles.len() == 2 && all_rings {
        return build_full_wall_face(builder, region, &info, &traversals);
    }
    if matches!(region.carrier, RegionCarrier::Cylinder { .. })
        && cycles.len() == 1
        && traversals[0].len() == 4
    {
        return build_cylinder_patch_face(builder, region, &info, &traversals[0]);
    }
    let mut detail = String::new();
    for (cycle_index, cycle_traversals) in traversals.iter().enumerate() {
        detail.push_str(&format!(" loop{cycle_index}:["));
        for traversal in cycle_traversals {
            let chain = &builder.chains[traversal.chain];
            let fit = ring_fit(builder, &info, chain);
            let points = builder.chain_points(chain);
            detail.push_str(&format!(
                "chain{}(n={},closed={},dz={:.2e},dr={:.2e},line={:.2e})",
                traversal.chain,
                chain.vertices.len(),
                chain.closed,
                fit.station_spread,
                fit.radial_spread,
                max_deviation_from_segment(&points),
            ));
        }
        detail.push(']');
    }
    Err(format!(
        "mesh_regions_to_brep: region {} ({}) has an unsupported boundary layout \
         ({} loops); v1 rebuilds full walls with two perpendicular rings and \
         four-sided cylinder patches;{detail}",
        region.id,
        region.carrier.kind(),
        cycles.len()
    ))
}

/// Full revolve wall bounded by two axis-perpendicular rings — the exact
/// `make_cylinder_brep` layout: seam edge used twice, one circle edge per
/// ring shared with the neighboring face.
fn build_full_wall_face(
    builder: &mut RegionBrepBuilder,
    region: &MeshRegion,
    info: &RevolveInfo,
    traversals: &[Vec<Traversal>],
) -> Result<FaceRecord, String> {
    if info.sense != 1 {
        return Err(format!(
            "mesh_regions_to_brep: region {} is a cavity wall (sense -1); inner revolve \
             walls are not supported in v1",
            region.id
        ));
    }
    let chain_a = traversals[0][0].chain;
    let chain_b = traversals[1][0].chain;
    let fit_a = ring_fit(builder, info, &builder.chains[chain_a]);
    let fit_b = ring_fit(builder, info, &builder.chains[chain_b]);
    for fit in [&fit_a, &fit_b] {
        if fit.station_spread > builder.tol || fit.radial_spread > builder.tol {
            return Err(format!(
                "mesh_regions_to_brep: region {} boundary ring is not an axis-perpendicular \
                 circle (station spread {:.3e}, radial spread {:.3e})",
                region.id, fit.station_spread, fit.radial_spread
            ));
        }
    }
    let (bottom_chain, bottom_fit, top_chain, top_fit) = if fit_a.station <= fit_b.station {
        (chain_a, fit_a, chain_b, fit_b)
    } else {
        (chain_b, fit_b, chain_a, fit_a)
    };
    if top_fit.station - bottom_fit.station <= builder.tol {
        return Err(format!(
            "mesh_regions_to_brep: region {} wall rings coincide axially",
            region.id
        ));
    }
    for chain in [bottom_chain, top_chain] {
        if builder.chain_edges[chain].is_some() {
            return Err(format!(
                "mesh_regions_to_brep: region {} shares a ring with another revolve wall — \
                 not supported in v1",
                region.id
            ));
        }
    }

    let r0 = info.radius_at(bottom_fit.station);
    let r1 = info.radius_at(top_fit.station);
    if !(r0 > builder.tol) || !(r1 > builder.tol) {
        return Err(format!(
            "mesh_regions_to_brep: region {} wall touches its apex/axis",
            region.id
        ));
    }
    let base = info.axis_point_at(bottom_fit.station);
    let top = info.axis_point_at(top_fit.station);
    let x_axis = info.axis.perpendicular()?;
    let generatrix = make_line(base.add(x_axis.scale(r0)), top.add(x_axis.scale(r1)))?;
    let surface = make_revolution(base, info.axis, &generatrix, std::f64::consts::TAU)?;

    let bottom_circle = surface.iso_curve_v(0.0)?;
    let top_circle = surface.iso_curve_v(1.0)?;
    let seam = surface.iso_curve_u(0.0)?;
    let bottom_vertex = builder.allocate_id();
    builder.vertices.push(VertexRecord {
        id: bottom_vertex,
        point: bottom_circle.evaluate(0.0)?,
    });
    let top_vertex = builder.allocate_id();
    builder.vertices.push(VertexRecord {
        id: top_vertex,
        point: top_circle.evaluate(0.0)?,
    });
    let bottom_edge = builder.push_edge(EdgeRecord {
        id: 0,
        curve: bottom_circle,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: bottom_vertex,
        end_vertex_id: bottom_vertex,
        degenerate: false,
        name: None,
    });
    let top_edge = builder.push_edge(EdgeRecord {
        id: 0,
        curve: top_circle,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: top_vertex,
        end_vertex_id: top_vertex,
        degenerate: false,
        name: None,
    });
    let seam_edge = builder.push_edge(EdgeRecord {
        id: 0,
        curve: seam,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: bottom_vertex,
        end_vertex_id: top_vertex,
        degenerate: false,
        name: None,
    });
    builder.chain_edges[bottom_chain] = Some(ChainEdgeInfo {
        edge_id: bottom_edge,
        curve_along_chain: true,
        ring: Some(RingClaim {
            axis_point: base,
            axis: info.axis,
            wall_forward: true,
        }),
    });
    builder.chain_edges[top_chain] = Some(ChainEdgeInfo {
        edge_id: top_edge,
        curve_along_chain: true,
        ring: Some(RingClaim {
            axis_point: top,
            axis: info.axis,
            wall_forward: false,
        }),
    });

    let coedges = vec![
        CoedgeRecord {
            id: builder.allocate_id(),
            edge_id: bottom_edge,
            forward: true,
            pcurve: parameter_segment(0.0, 0.0, 1.0, 0.0)?,
        },
        CoedgeRecord {
            id: builder.allocate_id(),
            edge_id: seam_edge,
            forward: true,
            pcurve: parameter_segment(1.0, 0.0, 1.0, 1.0)?,
        },
        CoedgeRecord {
            id: builder.allocate_id(),
            edge_id: top_edge,
            forward: false,
            pcurve: parameter_segment(1.0, 1.0, 0.0, 1.0)?,
        },
        CoedgeRecord {
            id: builder.allocate_id(),
            edge_id: seam_edge,
            forward: false,
            pcurve: parameter_segment(0.0, 1.0, 0.0, 0.0)?,
        },
    ];
    let loop_id = builder.allocate_id();
    let face_id = builder.allocate_id();
    Ok(FaceRecord {
        id: face_id,
        surface,
        same_sense: true,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: None,
    })
}

enum PatchRole {
    BottomArc,
    TopArc,
    RulingU0,
    RulingU1,
}

/// Partial cylinder patch (fillet blend): one loop of two axis-perpendicular
/// arcs joined by two straight rulings, rebuilt on a full-sweep cylinder
/// carrier with iso-parameter edges over the patch's u range.
fn build_cylinder_patch_face(
    builder: &mut RegionBrepBuilder,
    region: &MeshRegion,
    info: &RevolveInfo,
    traversals: &[Traversal],
) -> Result<FaceRecord, String> {
    let radius = info.radius_offset;
    // Classify the four chains.  Alternation (arc, ruling, arc, ruling) is
    // implied by the corner checks below: each ruling must land exactly on
    // an arc endpoint.
    let mut arcs: Vec<(usize, f64)> = Vec::new();
    let mut rulings: Vec<usize> = Vec::new();
    for traversal in traversals {
        let chain = &builder.chains[traversal.chain];
        if chain.closed {
            return Err(format!(
                "mesh_regions_to_brep: region {} patch loop contains a closed ring",
                region.id
            ));
        }
        let fit = ring_fit(builder, info, chain);
        let points = builder.chain_points(chain);
        let is_arc = fit.station_spread <= builder.tol && fit.radial_spread <= builder.tol;
        let straight = max_deviation_from_segment(&points) <= builder.tol;
        let axis_parallel = if straight {
            let direction = points
                .last()
                .unwrap()
                .sub(points[0])
                .normalized()
                .unwrap_or_default();
            direction.cross(info.axis).length() <= 1e-3
        } else {
            false
        };
        if is_arc {
            arcs.push((traversal.chain, fit.station));
        } else if straight && axis_parallel {
            rulings.push(traversal.chain);
        } else {
            return Err(format!(
                "mesh_regions_to_brep: region {} patch boundary chain is neither an \
                 axis-perpendicular arc nor an axial ruling (v1)",
                region.id
            ));
        }
    }
    if arcs.len() != 2 || rulings.len() != 2 {
        return Err(format!(
            "mesh_regions_to_brep: region {} patch needs two arcs and two rulings \
             (found {} arcs, {} rulings)",
            region.id,
            arcs.len(),
            rulings.len()
        ));
    }
    let (bottom_chain, s0) = if arcs[0].1 <= arcs[1].1 {
        arcs[0]
    } else {
        arcs[1]
    };
    let (top_chain, s1) = if arcs[0].1 <= arcs[1].1 {
        arcs[1]
    } else {
        arcs[0]
    };
    if s1 - s0 <= builder.tol {
        return Err(format!(
            "mesh_regions_to_brep: region {} patch arcs coincide axially",
            region.id
        ));
    }
    for chain in [bottom_chain, top_chain, rulings[0], rulings[1]] {
        if builder.chain_edges[chain].is_some() {
            return Err(format!(
                "mesh_regions_to_brep: region {} patch chain already claimed by another \
                 curved region — not supported in v1",
                region.id
            ));
        }
    }

    // Exact bottom arc along the chain's stored order.
    let center = info.axis_point_at(s0);
    let bottom_points = builder.chain_points(&builder.chains[bottom_chain]);
    let x_axis = info.radial(bottom_points[0]).normalized()?;
    let y_ccw = info.axis.cross(x_axis);
    let mut swept = 0.0_f64;
    let mut previous = 0.0_f64;
    for &p in &bottom_points[1..] {
        let radial = info.radial(p);
        let angle = radial.dot(y_ccw).atan2(radial.dot(x_axis));
        swept += wrap_to_pi(angle - previous);
        previous = angle;
    }
    if swept.abs() <= 1e-9 || swept.abs() >= std::f64::consts::TAU - 1e-9 {
        return Err(format!(
            "mesh_regions_to_brep: region {} patch arc sweep {swept:.3e} out of range",
            region.id
        ));
    }
    // The patch lies on a FULL-sweep cylinder carrier, trimmed by its four
    // edges — the carrier the hybrid builder and a revolved feature use, so it
    // recognizes as a cylinder and writes to STEP as CYLINDRICAL_SURFACE. (An
    // extruded arc is the same point set but recognizes as a partial-sweep
    // revolution, which reads "surface of revolution" and exports as a
    // B-spline.) The u = 0 meridian is the arc end from which the
    // counter-clockwise turn about the axis covers the patch, so the patch is
    // u in [0, u_end] and no edge crosses the seam.
    let bottom_first = builder.chains[bottom_chain].vertices[0];
    let bottom_last = *builder.chains[bottom_chain].vertices.last().unwrap();
    let start_is_chain_first = swept >= 0.0;
    let (start_welded, end_welded) = if start_is_chain_first {
        (bottom_first, bottom_last)
    } else {
        (bottom_last, bottom_first)
    };
    let meridian = info
        .radial(builder.data.verts[start_welded])
        .normalized()?;
    let generatrix = make_line(
        center.add(meridian.scale(radius)),
        info.axis_point_at(s1).add(meridian.scale(radius)),
    )?;
    let surface = make_revolution(center, info.axis, &generatrix, std::f64::consts::TAU)?;
    let spans = distinct_interior_knots(&surface.knots_u) + 1;
    let u_end = circle_angle_to_parameter(spans, std::f64::consts::TAU, swept.abs());
    let bottom_curve = sub_curve(&surface.iso_curve_v(0.0)?, 0.0, u_end)?;
    let top_curve = sub_curve(&surface.iso_curve_v(1.0)?, 0.0, u_end)?;
    let ruling_line_u0 = surface.iso_curve_u(0.0)?;
    let ruling_line_u1 = surface.iso_curve_u(u_end)?;

    let geometry_tolerance = builder.tol.max(1e-9 * builder.data.diag);
    let profile_start = bottom_curve.evaluate(0.0)?;
    let profile_end = bottom_curve.evaluate(1.0)?;
    if profile_start
        .sub(builder.data.verts[start_welded])
        .length()
        > geometry_tolerance
        || profile_end.sub(builder.data.verts[end_welded]).length() > geometry_tolerance
    {
        return Err(format!(
            "mesh_regions_to_brep: region {} exact arc misses its mesh corners",
            region.id
        ));
    }
    let top_first = builder.chains[top_chain].vertices[0];
    let top_last = *builder.chains[top_chain].vertices.last().unwrap();
    let top_start_point = top_curve.evaluate(0.0)?;
    let top_along =
        if builder.data.verts[top_first].sub(top_start_point).length() <= geometry_tolerance {
            true
        } else if builder.data.verts[top_last].sub(top_start_point).length() <= geometry_tolerance {
            false
        } else {
            return Err(format!(
                "mesh_regions_to_brep: region {} top arc corners do not sit above the bottom arc",
                region.id
            ));
        };
    let (top_start_welded, top_end_welded) = if top_along {
        (top_first, top_last)
    } else {
        (top_last, top_first)
    };

    // Match each ruling chain to u=0 (arc start) or u=u_end (arc end).
    let ruling_of = |chain_id: usize| -> Result<(PatchRole, usize, usize), String> {
        let chain = &builder.chains[chain_id];
        let first = chain.vertices[0];
        let last = *chain.vertices.last().unwrap();
        let (bottom_welded, top_welded) =
            if info.station(builder.data.verts[first]) <= info.station(builder.data.verts[last]) {
                (first, last)
            } else {
                (last, first)
            };
        let p_bottom = builder.data.verts[bottom_welded];
        if p_bottom.sub(profile_start).length() <= geometry_tolerance {
            Ok((PatchRole::RulingU0, bottom_welded, top_welded))
        } else if p_bottom.sub(profile_end).length() <= geometry_tolerance {
            Ok((PatchRole::RulingU1, bottom_welded, top_welded))
        } else {
            Err(format!(
                "mesh_regions_to_brep: region {} ruling does not meet the arc corners",
                region.id
            ))
        }
    };
    let (role_a, ruling_a_bottom, ruling_a_top) = ruling_of(rulings[0])?;
    let (role_b, ruling_b_bottom, ruling_b_top) = ruling_of(rulings[1])?;
    if matches!(role_a, PatchRole::RulingU0) == matches!(role_b, PatchRole::RulingU0) {
        return Err(format!(
            "mesh_regions_to_brep: region {} rulings both land on the same arc corner",
            region.id
        ));
    }

    // Create the four shared edges.
    let register = |builder: &mut RegionBrepBuilder,
                    chain_id: usize,
                    curve: NurbsCurve,
                    start_welded: usize,
                    end_welded: usize,
                    curve_along_chain: bool|
     -> u64 {
        let start_vertex_id = builder.welded_vertex_id(start_welded);
        let end_vertex_id = builder.welded_vertex_id(end_welded);
        let edge_id = builder.push_edge(EdgeRecord {
            id: 0,
            curve,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id,
            end_vertex_id,
            degenerate: false,
            name: None,
        });
        builder.chain_edges[chain_id] = Some(ChainEdgeInfo {
            edge_id,
            curve_along_chain,
            ring: None,
        });
        edge_id
    };
    register(
        builder,
        bottom_chain,
        bottom_curve,
        start_welded,
        end_welded,
        start_is_chain_first,
    );
    register(
        builder,
        top_chain,
        top_curve,
        top_start_welded,
        top_end_welded,
        top_along,
    );
    let (u0_chain, u0_bottom, u0_top, u1_chain, u1_bottom, u1_top) =
        if matches!(role_a, PatchRole::RulingU0) {
            (
                rulings[0],
                ruling_a_bottom,
                ruling_a_top,
                rulings[1],
                ruling_b_bottom,
                ruling_b_top,
            )
        } else {
            (
                rulings[1],
                ruling_b_bottom,
                ruling_b_top,
                rulings[0],
                ruling_a_bottom,
                ruling_a_top,
            )
        };
    register(
        builder,
        u0_chain,
        ruling_line_u0,
        u0_bottom,
        u0_top,
        builder.chains[u0_chain].vertices[0] == u0_bottom,
    );
    register(
        builder,
        u1_chain,
        ruling_line_u1,
        u1_bottom,
        u1_top,
        builder.chains[u1_chain].vertices[0] == u1_bottom,
    );

    // Patch loop in the region's own walk order (guarantees opposite edge
    // senses against the planar neighbors' walks).
    let role_of = |chain_id: usize| -> PatchRole {
        if chain_id == bottom_chain {
            PatchRole::BottomArc
        } else if chain_id == top_chain {
            PatchRole::TopArc
        } else if chain_id == u0_chain {
            PatchRole::RulingU0
        } else {
            PatchRole::RulingU1
        }
    };
    let mut coedges = Vec::with_capacity(4);
    for traversal in traversals {
        let infoc = builder.chain_edges[traversal.chain]
            .as_ref()
            .expect("patch chains registered above");
        let forward = traversal.forward_along_chain == infoc.curve_along_chain;
        let edge_id = infoc.edge_id;
        let (from, to) = match role_of(traversal.chain) {
            PatchRole::BottomArc => ((0.0, 0.0), (u_end, 0.0)),
            PatchRole::TopArc => ((0.0, 1.0), (u_end, 1.0)),
            PatchRole::RulingU0 => ((0.0, 0.0), (0.0, 1.0)),
            PatchRole::RulingU1 => ((u_end, 0.0), (u_end, 1.0)),
        };
        let pcurve = if forward {
            parameter_segment(from.0, from.1, to.0, to.1)?
        } else {
            parameter_segment(to.0, to.1, from.0, from.1)?
        };
        coedges.push(CoedgeRecord {
            id: builder.allocate_id(),
            edge_id,
            forward,
            pcurve,
        });
    }
    let loop_id = builder.allocate_id();
    let face_id = builder.allocate_id();
    // The revolved carrier's normal is radial outward; a concave patch
    // (cavity side, sense -1) takes the face's reversed sense.
    Ok(FaceRecord {
        id: face_id,
        surface,
        same_sense: info.sense == 1,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: None,
    })
}

/// Distinct knot values strictly inside a clamped knot vector.
fn distinct_interior_knots(knots: &[f64]) -> usize {
    let (Some(&first), Some(&last)) = (knots.first(), knots.last()) else {
        return 0;
    };
    let mut count = 0;
    let mut previous = first;
    for &knot in knots {
        if knot - previous > KNOT_IDENTITY_TOL && last - knot > KNOT_IDENTITY_TOL {
            count += 1;
            previous = knot;
        }
    }
    count
}

/// The exact piece of `curve` over `[t0, t1]` (inside its [0, 1] domain),
/// reparameterized affinely onto [0, 1]: `piece(s) = curve(t0 + s·(t1 − t0))`.
fn sub_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    let [start, end] = curve.domain()?;
    let mut piece = curve.clone();
    if t1 < end - KNOT_IDENTITY_TOL {
        piece = piece.split(t1)?.0;
    }
    if t0 > start + KNOT_IDENTITY_TOL {
        piece = piece.split(t0)?.1;
    }
    let span = t1 - t0;
    let knots = piece.knots.iter().map(|knot| (knot - t0) / span).collect();
    NurbsCurve::new(piece.degree, knots, piece.control_points.clone())
}

/// Planar region: one trimmed plane face; loops come from the region's
/// ordered boundary walks, edges are shared chain curves, pcurves are exact
/// affine projections.
pub(super) fn build_planar_region_face(
    builder: &mut RegionBrepBuilder,
    region_id: u32,
    origin: Vec3,
    normal: Vec3,
    cycles: &[Vec<usize>],
) -> Result<FaceRecord, String> {
    if cycles.is_empty() {
        return Err(format!(
            "mesh_regions_to_brep: planar region {region_id} has no boundary"
        ));
    }
    let x_axis = normal.perpendicular()?;
    let y_axis = normal.cross(x_axis);

    // Resolve every loop to (edge id, forward) pairs first; the plane's
    // extents must cover all edge control points before pcurves exist.
    let mut loops: Vec<(Vec<(u64, bool)>, f64)> = Vec::new();
    for cycle in cycles {
        let traversals = builder.cycle_traversals(cycle)?;
        let mut entries = Vec::with_capacity(traversals.len());
        for traversal in &traversals {
            if builder.chain_edges[traversal.chain].is_none() {
                builder.ensure_open_chain_edge(traversal.chain)?;
            }
            let chain_info = builder.chain_edges[traversal.chain].as_ref().unwrap();
            let forward = if let Some(claim) = &chain_info.ring {
                let circulation = builder.cycle_circulation(cycle, claim.axis_point, claim.axis)?;
                let forward = circulation > 0.0;
                if forward == claim.wall_forward {
                    return Err(format!(
                        "mesh_regions_to_brep: planar region {region_id} traverses a wall \
                         ring in the wall's own direction (non-manifold orientation)"
                    ));
                }
                forward
            } else {
                traversal.forward_along_chain == chain_info.curve_along_chain
            };
            entries.push((chain_info.edge_id, forward));
        }
        // Signed area of the walk polygon in the outward plane frame:
        // positive = outer loop, negative = hole.
        let mut area = 0.0;
        for index in 0..cycle.len() {
            let a = builder.data.verts[cycle[index]].sub(origin);
            let b = builder.data.verts[cycle[(index + 1) % cycle.len()]].sub(origin);
            let (ax, ay) = (a.dot(x_axis), a.dot(y_axis));
            let (bx, by) = (b.dot(x_axis), b.dot(y_axis));
            area += 0.5 * (ax * by - bx * ay);
        }
        loops.push((entries, area));
    }
    loops.sort_by(|a, b| b.1.total_cmp(&a.1));
    if !(loops[0].1 > 0.0) || loops.iter().skip(1).any(|entry| !(entry.1 < 0.0)) {
        return Err(format!(
            "mesh_regions_to_brep: planar region {region_id} loops do not split into one \
             outer boundary plus holes"
        ));
    }

    // Plane extents over every referenced curve's control hull, with margin
    // so evaluated uv never clamps at the domain boundary.
    let mut min_u = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut min_v = f64::INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for (entries, _) in &loops {
        for &(edge_id, _) in entries {
            for control in &builder.edge_curve(edge_id).control_points {
                let delta = control.point()?.sub(origin);
                let u = delta.dot(x_axis);
                let v = delta.dot(y_axis);
                min_u = min_u.min(u);
                max_u = max_u.max(u);
                min_v = min_v.min(v);
                max_v = max_v.max(v);
            }
        }
    }
    let extent = (max_u - min_u).max(max_v - min_v);
    if !(extent > 0.0) || !extent.is_finite() {
        return Err(format!(
            "mesh_regions_to_brep: planar region {region_id} has a degenerate boundary"
        ));
    }
    let margin = (extent * 0.05).max(1e-6 * builder.data.diag);
    let plane_origin = origin
        .add(x_axis.scale(min_u - margin))
        .add(y_axis.scale(min_v - margin));
    let surface = make_plane(
        plane_origin,
        x_axis,
        y_axis,
        (max_u - min_u) + 2.0 * margin,
        (max_v - min_v) + 2.0 * margin,
    )?;

    let mut loop_records = Vec::with_capacity(loops.len());
    for (entries, _) in &loops {
        let mut coedges = Vec::with_capacity(entries.len());
        for &(edge_id, forward) in entries {
            let mapped = affine_pcurve(builder.edge_curve(edge_id), plane_origin, x_axis, y_axis)?;
            let pcurve = if forward { mapped } else { mapped.reversed()? };
            coedges.push(CoedgeRecord {
                id: builder.allocate_id(),
                edge_id,
                forward,
                pcurve,
            });
        }
        let loop_id = builder.allocate_id();
        loop_records.push(LoopRecord {
            id: loop_id,
            coedges,
        });
    }
    let face_id = builder.allocate_id();
    Ok(FaceRecord {
        id: face_id,
        surface,
        same_sense: true,
        loops: loop_records,
        name: None,
    })
}
