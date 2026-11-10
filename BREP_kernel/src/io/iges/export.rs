//! Kernel `BrepSolid` → IGES 5.3 Manifold Solid B-Rep Objects.
//!
//! Each solid becomes a **186** whose shells (**514**) hold faces (**510**)
//! bounded by loops (**508**) over one edge list (**504**) and one vertex list
//! (**502**), so neighbouring faces share their edges as they do in the
//! kernel. A carrier the kernel recognises as analytic is written as the
//! analytic entity, form 1 (**190** plane, **192** cylinder, **194** cone,
//! **196** sphere, **198** torus, located by **116** points and **123**
//! directions); any other carrier is a **128** B-spline. Edge curves are
//! **126** B-splines, exact for the kernel's NURBS. A body's name goes in its
//! directory-entry label and a **406** form 15 name property, and its colour,
//! when given, in a **314** colour definition its directory entry points to.
//!
//! Every edge use on a plane or B-spline carrier also carries its parameter
//! curve (a **126** in `(u, v, 0)`, running along the loop), so a reader
//! never has to project an edge onto its face: on a plane it is the edge curve
//! in the plane's own axes, exact; on a 128 it is the kernel's stored pcurve,
//! whose parameter space the 128 shares; on a 192 cylinder, 196 sphere or 198
//! torus it is the STEP writer's verified pcurve (`build_pcurve`) with its
//! angles in degrees, the unit IGES readers take them in. A cone, and any use
//! whose pcurve misses the export band, carries none and is projected by the
//! reader.
//!
//! [`export_iges_assembly`] writes a document's product structure: each part
//! once, its solids as **186**s inside a **308** subfigure definition, and each
//! placement of it as a **408** instance moved by a **124**, named by its
//! instance label, nested as deep as the structure goes.
//!
//! Orientation follows the importer's reading of the standard: a 508 runs
//! counter-clockwise about its SURFACE's normal, and a face's orientation flag
//! in its 514 says whether the face's outward normal agrees with that.

use crate::curve::curve_to_plane_parameters;
use crate::step::edge_subcurve;
use crate::step::pcurve::{build_pcurve, EmittedCurve, EmittedFrame, EmittedSurface, Pcurve2d};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord};
use crate::{AnalyticSurface, KernelTolerances, NurbsCurve, NurbsSurface, Vec3, Vec4};
use rustc_hash::FxHashMap as HashMap;

use super::entities::{curve_to_126, surface_to_128};
use super::writer::{
    hollerith, real_field, GlobalParams, IgesEntity, IgesWriter, STATUS_DEFINITION, STATUS_DEPENDENT,
    STATUS_INDEPENDENT,
};
use crate::{StepAssemblyExport, StepColors};

/// One body to write: its name, the solid, and an optional sRGB colour.
pub struct IgesBody<'a> {
    pub name: String,
    pub solid: &'a BrepSolid,
    pub color: Option<[u8; 3]>,
}

/// Serialize solids as an IGES 5.3 document of Manifold Solid B-Rep Objects,
/// every body under `name`.
///
/// `unit` is one of `millimeter|meter|centimeter|inch|foot` (default
/// millimetre); coordinates are written verbatim, so callers pass the unit the
/// kernel geometry is expressed in (millimetres in this kernel).
pub fn export_iges(
    solids: &[BrepSolid],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    let bodies: Vec<IgesBody<'_>> = solids
        .iter()
        .map(|solid| IgesBody { name: name.to_string(), solid, color: None })
        .collect();
    export_iges_bodies(&bodies, name, unit, timestamp)
}

/// [`export_iges`] with a name and colour per body.
pub fn export_iges_bodies(
    bodies: &[IgesBody<'_>],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if bodies.is_empty() {
        return Err("iges_export: no solids to export".into());
    }
    let mut writer = IgesWriter::new();
    writer.add_start_line(&format!("IGES export of '{name}' from the BREP kernel."));
    for body in bodies {
        write_solid(&mut writer, body, STATUS_INDEPENDENT)?;
    }
    Ok(finish(writer, name, unit, timestamp))
}

/// A document's product structure as IGES: every product other than the root
/// as a 308 subfigure definition holding its own solids (in its own frame) and
/// the instances placed inside it, every occurrence as a 408 instance whose 124
/// carries its placement and whose 406 name is its designator, and the root's
/// own solids and instances standing on their own. Solids are named by their
/// part-local scene names and coloured from `colors`, as the STEP writer does.
pub fn export_iges_assembly(
    assembly: &StepAssemblyExport,
    colors: &StepColors,
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    let root = assembly.root()?;
    let mut writer = IgesWriter::new();
    writer.add_start_line(&format!("IGES export of '{name}' from the BREP kernel."));
    let mut subfigures: Vec<Option<(i64, usize)>> = vec![None; assembly.products.len()];
    // A part's colours resolve through every occurrence namespace that reaches
    // it, first in sorted order, as the STEP writer resolves them.
    let prefixes = assembly.occurrence_prefixes(root);
    let context = AssemblyContext { assembly, colors, root, prefixes: &prefixes };
    write_product(&mut writer, &context, root, &mut subfigures)?;
    Ok(finish(writer, name, unit, timestamp))
}

/// Write `index` (children first) and return nothing for the root, whose
/// entities stand on their own, or its 308's pointer and nesting depth.
struct AssemblyContext<'a> {
    assembly: &'a StepAssemblyExport,
    colors: &'a StepColors,
    root: usize,
    prefixes: &'a [Vec<String>],
}

fn write_product(
    writer: &mut IgesWriter,
    context: &AssemblyContext<'_>,
    index: usize,
    subfigures: &mut Vec<Option<(i64, usize)>>,
) -> Result<(), String> {
    let (assembly, root) = (context.assembly, context.root);
    if subfigures[index].is_some() {
        return Ok(());
    }
    let is_root = index == root;
    let status = if is_root { STATUS_INDEPENDENT } else { STATUS_DEPENDENT };
    let mut members = Vec::new();
    let mut depth = 1;
    let product = &assembly.products[index];
    for (body_name, solid) in &product.bodies {
        let color = context.colors.resolve(&context.prefixes[index], body_name);
        let body = IgesBody { name: body_name.clone(), solid, color };
        members.push(write_solid(writer, &body, status)?);
    }
    for occurrence in assembly.occurrences.iter().filter(|occurrence| occurrence.parent == index) {
        write_product(writer, context, occurrence.child, subfigures)?;
        let (definition, child_depth) =
            subfigures[occurrence.child].ok_or("iges_export: an occurrence places the root product")?;
        depth = depth.max(child_depth + 1);
        let mut matrix = IgesEntity::new(
            124,
            // The top three rows, row-major: R11 R12 R13 T1 R21 ... T3.
            occurrence.placement[..12].iter().map(|&value| real_field(value)).collect(),
        );
        matrix.status = STATUS_DEPENDENT;
        let matrix = writer.add_entity(matrix);
        let mut params = vec![definition.to_string(), real_field(0.0), real_field(0.0), real_field(0.0), real_field(1.0)];
        if !occurrence.designator.is_empty() {
            let mut label = IgesEntity::new(406, vec!["1".into(), hollerith(&occurrence.designator)]);
            label.form = 15;
            let property = writer.add_entity(label);
            params.extend(["0".into(), "1".into(), property.to_string()]);
        }
        let mut instance = IgesEntity::new(408, params);
        instance.transform = matrix;
        instance.status = status;
        instance.label = occurrence.designator.chars().filter(|c| !c.is_whitespace()).take(8).collect();
        members.push(writer.add_entity(instance));
    }
    if is_root {
        // The root is the file itself; mark it written so a cycle cannot recur.
        subfigures[index] = Some((0, 0));
        return Ok(());
    }
    let mut params = vec![depth.to_string(), hollerith(&product.name), members.len().to_string()];
    params.extend(members.iter().map(i64::to_string));
    let mut definition = IgesEntity::new(308, params);
    definition.status = STATUS_DEFINITION;
    definition.label = product.name.chars().filter(|c| !c.is_whitespace()).take(8).collect();
    subfigures[index] = Some((writer.add_entity(definition), depth));
    Ok(())
}

fn finish(writer: IgesWriter, name: &str, unit: &str, timestamp: &str) -> String {
    let (units_flag, units_name) = units_for(unit);
    let global = GlobalParams {
        product_id: if name.is_empty() { "PART".into() } else { name.to_string() },
        file_name: format!("{}.igs", if name.is_empty() { "part" } else { name }),
        units_flag,
        units_name,
        timestamp: normalize_timestamp(timestamp),
        min_resolution: 1e-7,
    };
    writer.finish(global)
}

/// The shared vertex and edge lists of one solid, and where each record went.
struct Lists {
    vertex_list: i64,
    edge_list: i64,
    vertex_index: HashMap<u64, usize>,
    edge_index: HashMap<u64, usize>,
    /// The band a quadric pcurve must verify to, as the STEP writer's.
    band: f64,
}

/// Write one solid as a 186 with the given directory status; returns its
/// pointer.
pub(super) fn write_solid(writer: &mut IgesWriter, body: &IgesBody<'_>, status: &'static str) -> Result<i64, String> {
    let solid = body.solid;
    if solid.shells.is_empty() {
        return Err(format!("iges_export: solid '{}' has no shells", body.name));
    }
    let shells = write_shells(writer, solid)?;
    // The first shell bounds the solid; any others are its cavities.
    let mut params = vec![shells[0].to_string(), "1".into(), (shells.len() - 1).to_string()];
    for void in &shells[1..] {
        params.push(void.to_string());
        params.push("1".into());
    }
    // Name property, then the colour the directory entry will point to.
    let label: String = body.name.chars().filter(|c| !c.is_whitespace()).take(8).collect();
    let property = (!body.name.is_empty()).then(|| {
        let mut name = IgesEntity::new(406, vec!["1".into(), hollerith(&body.name)]);
        name.form = 15;
        writer.add_entity(name)
    });
    let color = body.color.map(|[r, g, b]| {
        let percent = |c: u8| real_field(f64::from(c) * 100.0 / 255.0);
        writer.add_entity(IgesEntity::new(
            314,
            vec![percent(r), percent(g), percent(b), hollerith(&body.name)],
        ))
    });
    if let Some(property) = property {
        params.extend(["0".into(), "1".into(), property.to_string()]);
    }
    let mut entity = IgesEntity::new(186, params);
    entity.label = label;
    entity.color = color.map_or(0, |pointer| -pointer);
    entity.status = status;
    Ok(writer.add_entity(entity))
}

/// Each shell of `solid` as a closed 514 over the solid's shared edge and
/// vertex lists, in shell order.
pub(super) fn write_shells(writer: &mut IgesWriter, solid: &BrepSolid) -> Result<Vec<i64>, String> {
    let lists = write_lists(writer, solid)?;
    let mut shells = Vec::with_capacity(solid.shells.len());
    for shell in &solid.shells {
        let mut faces = Vec::with_capacity(shell.faces.len() * 2);
        for face in &shell.faces {
            let (pointer, agrees) = write_face(writer, solid, &lists, face)?;
            faces.push(pointer.to_string());
            faces.push(flag(agrees));
        }
        let mut params = vec![shell.faces.len().to_string()];
        params.extend(faces);
        let mut entity = IgesEntity::new(514, params);
        entity.form = 1;
        shells.push(writer.add_entity(entity));
    }
    Ok(shells)
}

fn flag(value: bool) -> String {
    if value { "1" } else { "0" }.into()
}

fn write_lists(writer: &mut IgesWriter, solid: &BrepSolid) -> Result<Lists, String> {
    let mut vertex_index = HashMap::default();
    let mut vertex_params = vec![solid.vertices.len().to_string()];
    for (index, vertex) in solid.vertices.iter().enumerate() {
        vertex_index.insert(vertex.id, index + 1);
        vertex_params.extend([real_field(vertex.point.x), real_field(vertex.point.y), real_field(vertex.point.z)]);
    }
    let mut vertex_entity = IgesEntity::new(502, vertex_params);
    vertex_entity.form = 1;
    let vertex_list = writer.add_entity(vertex_entity);

    let mut edge_index = HashMap::default();
    let edges: Vec<&EdgeRecord> = solid.edges.iter().filter(|edge| !edge.degenerate).collect();
    let mut edge_params = vec![edges.len().to_string()];
    for (index, edge) in edges.iter().enumerate() {
        edge_index.insert(edge.id, index + 1);
        let curve = writer.add_entity(curve_to_126(&edge_subcurve(edge)?)?);
        let start = vertex_of(&vertex_index, edge.start_vertex_id)?;
        let end = vertex_of(&vertex_index, edge.end_vertex_id)?;
        edge_params.extend([
            curve.to_string(),
            vertex_list.to_string(),
            start.to_string(),
            vertex_list.to_string(),
            end.to_string(),
        ]);
    }
    let mut edge_entity = IgesEntity::new(504, edge_params);
    edge_entity.form = 1;
    let edge_list = writer.add_entity(edge_entity);
    let band = KernelTolerances::for_solid(solid, 1e-7).export_knit;
    Ok(Lists { vertex_list, edge_list, vertex_index, edge_index, band })
}

fn vertex_of(index: &HashMap<u64, usize>, id: u64) -> Result<usize, String> {
    index.get(&id).copied().ok_or_else(|| format!("iges_export: dangling vertex id {id}"))
}

/// Write one face; returns its DE pointer and whether its outward normal
/// agrees with its IGES surface's normal.
fn write_face(
    writer: &mut IgesWriter,
    solid: &BrepSolid,
    lists: &Lists,
    face: &FaceRecord,
) -> Result<(i64, bool), String> {
    if face.loops.is_empty() {
        return Err(format!("iges_export: face {} has no loops", face.id));
    }
    let (surface, reversed, carrier) = write_surface(writer, &face.surface)?;
    let agrees = face.same_sense != reversed;
    let mut loops = Vec::with_capacity(face.loops.len());
    for loop_record in &face.loops {
        loops.push(write_loop(writer, solid, lists, face, loop_record, agrees, &carrier)?);
    }
    // N loops, outer-loop flag (the kernel keeps its outer loop first).
    let mut params = vec![surface.to_string(), face.loops.len().to_string(), "1".into()];
    params.extend(loops.iter().map(i64::to_string));
    let mut entity = IgesEntity::new(510, params);
    entity.form = 1;
    Ok((writer.add_entity(entity), agrees))
}

fn write_loop(
    writer: &mut IgesWriter,
    solid: &BrepSolid,
    lists: &Lists,
    face: &FaceRecord,
    loop_record: &LoopRecord,
    agrees: bool,
    carrier: &Carrier,
) -> Result<i64, String> {
    // Kernel loops run counter-clockwise about the FACE's normal; a 508 runs
    // counter-clockwise about its surface's, so a disagreeing face's loops
    // are written backwards.
    let mut uses: Vec<(&CoedgeRecord, bool)> = loop_record.coedges.iter().map(|c| (c, c.forward)).collect();
    if !agrees {
        uses.reverse();
        for (_, forward) in &mut uses {
            *forward = !*forward;
        }
    }
    let mut entries = Vec::with_capacity(uses.len());
    for (coedge, forward) in uses {
        let edge_id = coedge.edge_id;
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == edge_id)
            .ok_or_else(|| format!("iges_export: dangling edge id {edge_id}"))?;
        let entry: Vec<String> = if edge.degenerate {
            // A collapsed edge is a vertex use: TYPE 1 into the vertex list.
            let vertex = vertex_of(&lists.vertex_index, edge.start_vertex_id)?;
            // No parameter curve: measured on the corpus, OpenCASCADE closes a
            // collapsed edge's (u, v) line itself and fails its own check on
            // one we supply (m0007, m0139, m0143 invalid with it, valid without).
            vec!["1".into(), lists.vertex_list.to_string(), vertex.to_string(), "1".into(), "0".into()]
        } else {
            let index = lists.edge_index.get(&edge_id).copied().ok_or("iges_export: edge missing from its list")?;
            let mut entry = vec!["0".into(), lists.edge_list.to_string(), index.to_string(), flag(forward)];
            match parameter_curve(carrier, &face.surface, lists.band, edge, coedge)? {
                Some(curve) => {
                    // A use's parameter curve runs along the loop, as OpenCASCADE
                    // writes and reads it: reversed where the use runs against
                    // its edge. The domain stays the edge curve's.
                    let curve = if forward { curve } else { curve.reversed()? };
                    let pointer = writer.add_entity(curve_to_126(&curve)?);
                    // K = 1: one curve, not isoparametric.
                    entry.extend(["1".into(), "0".into(), pointer.to_string()]);
                }
                None => entry.push("0".into()),
            }
            entry
        };
        entries.push(entry);
    }
    let mut params = vec![entries.len().to_string()];
    for entry in entries {
        params.extend(entry);
    }
    let mut entity = IgesEntity::new(508, params);
    entity.form = 1;
    Ok(writer.add_entity(entity))
}

/// What a face's carrier was written as, which decides its parameter curves.
enum Carrier {
    /// A 190: parameters are distances along these axes from the origin.
    Plane { origin: Vec3, x_axis: Vec3, y_axis: Vec3 },
    /// A 128 of the kernel surface itself: its parameters are the kernel's.
    Spline,
    /// A 192, 196 or 198, in the STEP entity's parameters with its angles in
    /// degrees: u always, v too on a sphere or torus.
    Quadric { surface: EmittedSurface, angular_v: bool },
    /// A carrier written without parameter curves: a 194, whose v the readers
    /// disagree on, or a torus a reader cannot tell u from v on.
    Unparameterized,
}

/// Radians to the degrees IGES analytic surfaces are parameterized in.
const DEGREES: f64 = 180.0 / std::f64::consts::PI;

/// The parameter curve of one edge use, in the written carrier's parameters
/// in the edge's direction over its 126's parameter range (`write_loop` turns
/// it to the use's).
fn parameter_curve(
    carrier: &Carrier,
    stored: &NurbsSurface,
    band: f64,
    edge: &EdgeRecord,
    coedge: &CoedgeRecord,
) -> Result<Option<NurbsCurve>, String> {
    match carrier {
        Carrier::Unparameterized => Ok(None),
        Carrier::Quadric { surface, angular_v } => {
            let curve = edge_subcurve(edge)?;
            let [t0, t1] = curve.domain()?;
            let oriented = if coedge.forward { coedge.pcurve.clone() } else { coedge.pcurve.reversed()? };
            let outcome = build_pcurve(stored, surface, &EmittedCurve::Spline { curve }, &oriented, band)?;
            let v_scale = if *angular_v { DEGREES } else { 1.0 };
            // Scaling the homogeneous coordinates scales the curve exactly.
            let scaled = |c: Vec4| Vec4 { x: c.x * DEGREES, y: c.y * v_scale, z: 0.0, w: c.w };
            Ok(match outcome.curve {
                Some(Pcurve2d::Line { point, vector }) => {
                    let at = |s: f64| Vec4 { x: point[0] + s * vector[0], y: point[1] + s * vector[1], z: 0.0, w: 1.0 };
                    Some(NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![scaled(at(t0)), scaled(at(t1))])?)
                }
                Some(Pcurve2d::Spline(curve)) => {
                    let points = curve.control_points.iter().map(|&c| scaled(c)).collect();
                    Some(NurbsCurve::new(curve.degree, curve.knots.clone(), points)?)
                }
                // A 2D circle is only proposed against a 3D circle, which the
                // IGES writer never emits; anything else goes without.
                Some(Pcurve2d::Circle { .. }) | None => None,
            })
        }
        // Projecting the edge curve's control points is affine, so the plane
        // curve is exact and keeps the edge's knots and weights.
        Carrier::Plane { origin, x_axis, y_axis } => {
            Ok(Some(curve_to_plane_parameters(&edge_subcurve(edge)?, *origin, *x_axis, *y_axis)?))
        }
        Carrier::Spline => {
            // The stored pcurve runs in loop direction, fraction-synchronised
            // with the edge (as the STEP writer reads it).
            let mut curve = if coedge.forward { coedge.pcurve.clone() } else { coedge.pcurve.reversed()? };
            let [q0, q1] = curve.domain()?;
            let [t0, t1] = edge_subcurve(edge)?.domain()?;
            if q1 <= q0 || t1 <= t0 {
                return Ok(None);
            }
            let scale = (t1 - t0) / (q1 - q0);
            for knot in &mut curve.knots {
                *knot = t0 + (*knot - q0) * scale;
            }
            Ok(Some(curve))
        }
    }
}

/// Write a carrier; returns its DE pointer, whether the written entity's
/// normal is the reverse of the kernel surface's du x dv, and what it is.
fn write_surface(writer: &mut IgesWriter, surface: &NurbsSurface) -> Result<(i64, bool, Carrier), String> {
    if let Some(analytic) = surface.analytic() {
        if let Some((pointer, reversed)) = write_analytic(writer, surface, analytic)? {
            let emitted = |frame: &crate::analytic_surface::RevolutionFrame| EmittedFrame {
                origin: frame.origin,
                x_axis: frame.x_axis,
                y_axis: frame.y_axis,
                axis: frame.axis,
                azimuth_sign: 1.0,
            };
            let carrier = match analytic {
                AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
                    let normal = u_dir.cross(*v_dir).normalized()?;
                    let x_axis = u_dir.normalized()?;
                    Carrier::Plane { origin: *origin, x_axis, y_axis: normal.cross(x_axis) }
                }
                // `write_analytic` wrote a 192 exactly when the radius is constant.
                AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }
                    if (rho1 - rho0).abs() <= 1e-9 * (1.0 + rho0.abs().max(rho1.abs())) =>
                {
                    Carrier::Quadric {
                        surface: EmittedSurface::Cylinder { frame: emitted(frame), radius: *rho0 },
                        angular_v: false,
                    }
                }
                AnalyticSurface::RuledRevolution { .. } => Carrier::Unparameterized,
                AnalyticSurface::Sphere { frame, radius } => Carrier::Quadric {
                    surface: EmittedSurface::Sphere { frame: emitted(frame), radius: *radius },
                    angular_v: true,
                },
                // When the inner equator's radius R - r equals the tube's r, a
                // parallel and a meridian are congruent circles, and OpenCASCADE
                // reads a pcurve along one as along the other (m0291: R = 1,
                // r = 0.5, an inner-equator edge taken 0.56 mm off). Such a torus
                // goes without, and the reader projects.
                AnalyticSurface::Torus { major_radius, minor_radius, .. }
                    if ((major_radius - minor_radius) - minor_radius).abs() <= 1e-6 * major_radius.abs().max(1.0) =>
                {
                    Carrier::Unparameterized
                }
                AnalyticSurface::Torus { frame, major_radius, minor_radius } => Carrier::Quadric {
                    surface: EmittedSurface::Torus {
                        frame: emitted(frame),
                        major_radius: *major_radius,
                        minor_radius: *minor_radius,
                    },
                    angular_v: true,
                },
                AnalyticSurface::Revolution { .. } => Carrier::Unparameterized,
            };
            return Ok((pointer, reversed, carrier));
        }
    }
    Ok((writer.add_entity(surface_to_128(surface)?), false, Carrier::Spline))
}

fn point_116(writer: &mut IgesWriter, p: Vec3) -> i64 {
    writer.add_entity(IgesEntity::new(116, vec![real_field(p.x), real_field(p.y), real_field(p.z)]))
}

fn direction_123(writer: &mut IgesWriter, d: Vec3) -> i64 {
    writer.add_entity(IgesEntity::new(123, vec![real_field(d.x), real_field(d.y), real_field(d.z)]))
}

/// The kernel surface's normal at the middle of its domain.
fn sampled_normal(surface: &NurbsSurface) -> Result<(Vec3, Vec3), String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    Ok((surface.evaluate(u, v)?, surface.normal(u, v)?))
}

fn write_analytic(
    writer: &mut IgesWriter,
    surface: &NurbsSurface,
    analytic: &AnalyticSurface,
) -> Result<Option<(i64, bool)>, String> {
    let (point, normal) = sampled_normal(surface)?;
    match analytic {
        AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
            let (Ok(n), Ok(x)) = (u_dir.cross(*v_dir).normalized(), u_dir.normalized()) else {
                return Ok(None);
            };
            // The plane is written with the kernel's own normal: no reversal.
            let location = point_116(writer, *origin);
            let normal = direction_123(writer, n);
            let reference = direction_123(writer, x);
            let mut entity = IgesEntity::new(190, vec![location.to_string(), normal.to_string(), reference.to_string()]);
            entity.form = 1;
            Ok(Some((writer.add_entity(entity), false)))
        }
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } => {
            let outward = {
                let d = point.sub(frame.origin);
                d.sub(frame.axis.scale(d.dot(frame.axis)))
            };
            let reversed = normal.dot(outward) < 0.0;
            let radius_scale = 1.0 + rho0.abs().max(rho1.abs());
            if (rho1 - rho0).abs() <= 1e-9 * radius_scale {
                if *rho0 <= 0.0 {
                    return Ok(None);
                }
                let location = point_116(writer, frame.origin);
                let axis = direction_123(writer, frame.axis);
                let reference = direction_123(writer, frame.x_axis);
                let mut entity = IgesEntity::new(
                    192,
                    vec![location.to_string(), axis.to_string(), real_field(*rho0), reference.to_string()],
                );
                entity.form = 1;
                return Ok(Some((writer.add_entity(entity), reversed)));
            }
            // As the STEP writer: a cone touching its apex keeps its NURBS, and
            // the axis is turned so the radius grows along it.
            let slope = (rho1 - rho0) / height;
            let apex_margin = 2.0 * (1e-4 * height.abs().max(1.0) + 1e-9) * slope.abs();
            if rho0.min(*rho1) <= apex_margin {
                return Ok(None);
            }
            let direction = if slope >= 0.0 { frame.axis } else { frame.axis.scale(-1.0) };
            let location = point_116(writer, frame.origin);
            let axis = direction_123(writer, direction);
            let reference = direction_123(writer, frame.x_axis);
            let mut entity = IgesEntity::new(
                194,
                vec![
                    location.to_string(),
                    axis.to_string(),
                    real_field(*rho0),
                    real_field(slope.abs().atan().to_degrees()),
                    reference.to_string(),
                ],
            );
            entity.form = 1;
            Ok(Some((writer.add_entity(entity), reversed)))
        }
        AnalyticSurface::Sphere { frame, radius } => {
            let reversed = normal.dot(point.sub(frame.origin)) < 0.0;
            let location = point_116(writer, frame.origin);
            let axis = direction_123(writer, frame.axis);
            let reference = direction_123(writer, frame.x_axis);
            let mut entity = IgesEntity::new(
                196,
                vec![location.to_string(), real_field(*radius), axis.to_string(), reference.to_string()],
            );
            entity.form = 1;
            Ok(Some((writer.add_entity(entity), reversed)))
        }
        AnalyticSurface::Torus { frame, major_radius, minor_radius } => {
            let tube_centre = {
                let d = point.sub(frame.origin);
                let radial = d.sub(frame.axis.scale(d.dot(frame.axis)));
                let Ok(direction) = radial.normalized() else {
                    return Ok(None);
                };
                frame.origin.add(direction.scale(*major_radius))
            };
            let reversed = normal.dot(point.sub(tube_centre)) < 0.0;
            let location = point_116(writer, frame.origin);
            let axis = direction_123(writer, frame.axis);
            let reference = direction_123(writer, frame.x_axis);
            let mut entity = IgesEntity::new(
                198,
                vec![
                    location.to_string(),
                    axis.to_string(),
                    real_field(*major_radius),
                    real_field(*minor_radius),
                    reference.to_string(),
                ],
            );
            entity.form = 1;
            Ok(Some((writer.add_entity(entity), reversed)))
        }
        AnalyticSurface::Revolution { .. } => Ok(None),
    }
}

fn units_for(unit: &str) -> (i64, String) {
    match unit.trim().to_ascii_lowercase().as_str() {
        "inch" | "inches" | "in" => (1, "IN".into()),
        "foot" | "feet" | "ft" => (4, "FT".into()),
        "meter" | "metre" | "m" => (6, "M".into()),
        "centimeter" | "centimetre" | "cm" => (10, "CM".into()),
        _ => (2, "MM".into()),
    }
}

fn normalize_timestamp(timestamp: &str) -> String {
    if timestamp.trim().is_empty() {
        "00000000.000000".into()
    } else {
        timestamp.to_string()
    }
}
