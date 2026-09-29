use crate::analytic_surface::{circumcenter, AnalyticSurface};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, VertexRecord};
use crate::{make_arc, KernelTolerances, NurbsCurve, NurbsSurface, Vec3};
use rustc_hash::FxHashMap as HashMap;

#[path = "step/pcurve.rs"]
mod pcurve;
use pcurve::{build_pcurve, EmittedCurve, EmittedFrame, EmittedSurface, Pcurve2d};
#[path = "step/pmi.rs"]
pub(crate) mod pmi;
pub use pmi::StepPmi;
#[path = "step/assembly.rs"]
pub mod assembly;
pub use assembly::{
    export_step_assembly, export_step_assembly_report, StepAssemblyExport, StepExportOccurrence,
    StepExportProduct,
};
#[path = "step/export_tree.rs"]
mod export_tree;
pub use export_tree::{assembly_export_tree, PART_ATTRIBUTES, PART_NUMBER};
#[path = "step/styles.rs"]
pub(crate) mod styles;
pub use styles::StepColors;

pub use crate::step_matrix::Mat4;
pub(crate) use crate::step_matrix::{mat4_mul, MAT4_IDENTITY};

pub(crate) fn transform_point(matrix: &Mat4, point: Vec3) -> Vec3 {
    crate::AffineTransform {
        elements: *matrix,
    }
    .point(point)
}

pub(crate) fn step_string(value: &str) -> String {
    value.replace('\'', "''")
}

pub(crate) fn real(value: f64) -> Result<String, String> {
    if !value.is_finite() {
        return Err(format!("export_step: non-finite number {value}"));
    }
    // Analytic frame axes come from cross products that can round to -0.0;
    // normalize so directions never print a negative zero component.
    if value == 0.0 {
        return Ok("0.".into());
    }
    if value.fract() == 0.0 && value.abs() < 1e15 {
        return Ok(format!("{value:.0}."));
    }
    let mut output = format!("{value:.15}");
    while output.ends_with('0') {
        output.pop();
    }
    if output.ends_with('.') {
        output.push('0');
    }
    if output == "-0.0" {
        output = "0.0".into();
    }
    Ok(output)
}

fn knot_runs(knots: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut values = Vec::new();
    let mut multiplicities = Vec::new();
    for &knot in knots {
        if values
            .last()
            .is_some_and(|previous: &f64| (*previous - knot).abs() <= 1e-12)
        {
            *multiplicities.last_mut().unwrap() += 1;
        } else {
            values.push(knot);
            multiplicities.push(1);
        }
    }
    (values, multiplicities)
}

pub(crate) fn edge_subcurve(edge: &EdgeRecord) -> Result<NurbsCurve, String> {
    let [start, end] = edge.curve.domain()?;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut curve = edge.curve.clone();
    if edge.t0 > start + epsilon && edge.t0 < end - epsilon {
        curve = curve.split(edge.t0)?.1;
    }
    let domain = curve.domain()?;
    if edge.t1 < domain[1] - epsilon && edge.t1 > domain[0] + epsilon {
        curve = curve.split(edge.t1)?.0;
    }
    Ok(curve)
}

#[derive(Default)]
pub(crate) struct StepWriter {
    lines: Vec<String>,
}

impl StepWriter {
    pub(crate) fn add(&mut self, body: impl Into<String>) -> usize {
        let id = self.lines.len() + 1;
        self.lines.push(format!("#{id}={};", body.into()));
        id
    }

    fn data(&self) -> String {
        self.lines.join("\n")
    }
}

pub(crate) fn write_point(writer: &mut StepWriter, point: Vec3) -> Result<usize, String> {
    Ok(writer.add(format!(
        "CARTESIAN_POINT('',({},{},{}))",
        real(point.x)?,
        real(point.y)?,
        real(point.z)?
    )))
}

pub(crate) fn id_list(ids: &[usize]) -> String {
    format!(
        "({})",
        ids.iter()
            .map(|id| format!("#{id}"))
            .collect::<Vec<_>>()
            .join(",")
    )
}

pub(crate) fn write_direction(writer: &mut StepWriter, direction: Vec3) -> Result<usize, String> {
    Ok(writer.add(format!(
        "DIRECTION('',({},{},{}))",
        real(direction.x)?,
        real(direction.y)?,
        real(direction.z)?
    )))
}

pub(crate) fn write_placement(
    writer: &mut StepWriter,
    origin: Vec3,
    axis: Vec3,
    ref_direction: Vec3,
) -> Result<usize, String> {
    let origin = write_point(writer, origin)?;
    let axis = write_direction(writer, axis)?;
    let ref_direction = write_direction(writer, ref_direction)?;
    Ok(writer.add(format!(
        "AXIS2_PLACEMENT_3D('',#{origin},#{axis},#{ref_direction})"
    )))
}

/// Emit the analytic AP242 surface entity (PLANE / CYLINDRICAL_SURFACE /
/// CONICAL_SURFACE / SPHERICAL_SURFACE / TOROIDAL_SURFACE) for a recognized
/// carrier, or `None` when the surface must stay a B-spline. The second value
/// reports whether the STEP-standard orientation of the emitted entity (plane
/// normal along the placement axis; revolution normal outward) is the REVERSE
/// of the stored NURBS orientation, so ADVANCED_FACE can invert `same_sense`
/// and the face normal survives the round trip. The third describes the
/// emitted entity's own (u,v) parameterization for the pcurve writer.
fn write_analytic_surface(
    writer: &mut StepWriter,
    surface: &NurbsSurface,
) -> Result<Option<(usize, bool, EmittedSurface)>, String> {
    let Some(analytic) = surface.analytic() else {
        return Ok(None);
    };
    match analytic {
        AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        } => {
            // STEP planes are unbounded; the importer re-sizes the patch from
            // the face's edges, so only origin/normal/ref matter.
            let (Ok(normal), Ok(x_axis)) = (u_dir.cross(*v_dir).normalized(), u_dir.normalized())
            else {
                return Ok(None);
            };
            let placement = write_placement(writer, *origin, normal, x_axis)?;
            // The reader derives the plane's second parameter axis as
            // normal × ref_direction, so S(x,y) = origin + x·x̂ + y·ŷ with
            // those exact vectors — an orthonormal frame regardless of how the
            // stored patch scaled or sheared its own u_dir/v_dir.
            Ok(Some((
                writer.add(format!("PLANE('',#{placement})")),
                false,
                EmittedSurface::Plane {
                    origin: *origin,
                    x_axis,
                    y_axis: normal.cross(x_axis),
                },
            )))
        }
        AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } => {
            // The recognizer allows height < 0 (descending generatrix), whose
            // normal is the reverse of the standard outward convention the
            // importer reconstructs; report that so the face sense compensates.
            let flipped = *height < 0.0;
            let radius_scale = 1.0 + rho0.abs().max(rho1.abs());
            if (rho1 - rho0).abs() <= 1e-9 * radius_scale {
                if *rho0 <= 0.0 {
                    return Ok(None);
                }
                let placement = write_placement(writer, frame.origin, frame.axis, frame.x_axis)?;
                return Ok(Some((
                    writer.add(format!(
                        "CYLINDRICAL_SURFACE('',#{placement},{})",
                        real(*rho0)?
                    )),
                    flipped,
                    EmittedSurface::Cylinder {
                        frame: EmittedFrame {
                            origin: frame.origin,
                            x_axis: frame.x_axis,
                            y_axis: frame.y_axis,
                            axis: frame.axis,
                            azimuth_sign: 1.0,
                        },
                        radius: *rho0,
                    },
                )));
            }
            // Cone. The importer re-sizes the carrier from edge samples with a
            // 1e-4-scaled axial margin and clamps a negative extended-end
            // radius to zero — which BENDS the rebuilt slope when the apex sits
            // at an end of the face's axial range. Keep apex-touching cones as
            // exact NURBS instead of exporting a distorted carrier.
            let slope = (rho1 - rho0) / height;
            let apex_margin = 2.0 * (1e-4 * height.abs().max(1.0) + 1e-9) * slope.abs();
            if rho0.min(*rho1) <= apex_margin {
                return Ok(None);
            }
            // Orient the placement axis so the radius grows along +axis: STEP
            // semi-angles are positive. Radius at the placement origin stays
            // rho0 either way because the origin is on-axis at the v = 0 base.
            let axis = if slope >= 0.0 {
                frame.axis
            } else {
                frame.axis.scale(-1.0)
            };
            let placement = write_placement(writer, frame.origin, axis, frame.x_axis)?;
            Ok(Some((
                writer.add(format!(
                    "CONICAL_SURFACE('',#{placement},{},{})",
                    real(*rho0)?,
                    real(slope.abs().atan())?
                )),
                flipped,
                EmittedSurface::Cone {
                    frame: EmittedFrame {
                        origin: frame.origin,
                        x_axis: frame.x_axis,
                        // Reversing the placement axis reverses the derived
                        // second axis with it, so the emitted azimuth runs
                        // opposite the stored carrier's u.
                        y_axis: axis.cross(frame.x_axis),
                        axis,
                        azimuth_sign: if slope >= 0.0 { 1.0 } else { -1.0 },
                    },
                    radius: *rho0,
                    semi_angle: slope.abs().atan(),
                },
            )))
        }
        AnalyticSurface::Sphere { frame, radius } => {
            // Recognition template and importer reconstruction share the same
            // south-to-north meridian construction, so the rebuild is exact.
            let placement = write_placement(writer, frame.origin, frame.axis, frame.x_axis)?;
            Ok(Some((
                writer.add(format!(
                    "SPHERICAL_SURFACE('',#{placement},{})",
                    real(*radius)?
                )),
                false,
                EmittedSurface::Sphere {
                    frame: EmittedFrame {
                        origin: frame.origin,
                        x_axis: frame.x_axis,
                        y_axis: frame.y_axis,
                        axis: frame.axis,
                        azimuth_sign: 1.0,
                    },
                    radius: *radius,
                },
            )))
        }
        AnalyticSurface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => {
            let placement = write_placement(writer, frame.origin, frame.axis, frame.x_axis)?;
            Ok(Some((
                writer.add(format!(
                    "TOROIDAL_SURFACE('',#{placement},{},{})",
                    real(*major_radius)?,
                    real(*minor_radius)?
                )),
                false,
                EmittedSurface::Torus {
                    frame: EmittedFrame {
                        origin: frame.origin,
                        x_axis: frame.x_axis,
                        y_axis: frame.y_axis,
                        axis: frame.axis,
                        azimuth_sign: 1.0,
                    },
                    major_radius: *major_radius,
                    minor_radius: *minor_radius,
                },
            )))
        }
        // No SURFACE_OF_REVOLUTION reader exists yet; general revolutions keep
        // their exact NURBS form.
        AnalyticSurface::Revolution { .. } => Ok(None),
    }
}

/// A curve recognized as an exact `make_arc` product: a circular arc of
/// `radius` about `axis`, starting at angle 0 on `x_axis` and travelling
/// counterclockwise through `sweep` — exactly the CIRCLE parameterization the
/// importer trims between the edge's vertices.
struct CircularArc {
    center: Vec3,
    axis: Vec3,
    x_axis: Vec3,
    /// axis × x_axis — the second placement axis a STEP reader derives, so the
    /// emitted parameterization is C(a) = center + r(cos a·x̂ + sin a·ŷ).
    y_axis: Vec3,
    radius: f64,
    /// Total swept angle: the emitted entity's parameter range is [0, sweep].
    sweep: f64,
    /// Rational-quadratic span count of the kernel arc this was recognized
    /// from — the bridge from the emitted ANGLE back to the kernel's own
    /// parameter, which a fitted pcurve on a B-spline carrier needs.
    spans: usize,
}

fn curve_scale(curve: &NurbsCurve) -> f64 {
    curve
        .control_points
        .iter()
        .map(|p| (p.x.abs() / p.w).max(p.y.abs() / p.w).max(p.z.abs() / p.w))
        .fold(0.0, f64::max)
}

/// Homogeneous-net equality to a scale-relative tolerance (the same
/// reconstruction contract analytic_surface.rs uses): matching nets mean the
/// curves are the SAME exact rational arc, not merely close.
fn curves_match(a: &NurbsCurve, b: &NurbsCurve, scale: f64) -> bool {
    if a.degree != b.degree
        || a.knots.len() != b.knots.len()
        || a.control_points.len() != b.control_points.len()
    {
        return false;
    }
    if a.knots
        .iter()
        .zip(&b.knots)
        .any(|(x, y)| (x - y).abs() > 1e-12)
    {
        return false;
    }
    let tolerance = 1e-9 * scale.max(1.0);
    a.control_points
        .iter()
        .zip(&b.control_points)
        .all(|(p, q)| {
            (p.x - q.x).abs() <= tolerance
                && (p.y - q.y).abs() <= tolerance
                && (p.z - q.z).abs() <= tolerance
                && (p.w - q.w).abs() <= 1e-9
        })
}

/// Recognition by exact reconstruction: extract a candidate circle from three
/// curve points, rebuild it with `make_arc`, and demand the identical net.
/// Split subranges of a circle (whose knots are no longer the pristine
/// make_arc pattern) are rejected and honestly stay NURBS.
fn recognize_circular_arc(curve: &NurbsCurve) -> Option<CircularArc> {
    if curve.degree != 2
        || curve.control_points.len() < 3
        || curve.control_points.len() % 2 == 0
        || (curve.control_points.len() - 1) / 2 > 4
    {
        return None;
    }
    let [t0, t1] = curve.domain().ok()?;
    let at = |fraction: f64| curve.evaluate(t0 + (t1 - t0) * fraction);
    // Three points at < 74% of the sweep apart, so consecutive pairs subtend
    // less than pi and the cross product below gives the travel direction.
    let p0 = at(0.0).ok()?;
    let pa = at(0.35).ok()?;
    let pb = at(0.7).ok()?;
    let center = circumcenter(p0, pa, pb)?;
    let radial = p0.sub(center);
    let radius = radial.length();
    let scale = curve_scale(curve);
    if radius <= 1e-9 * scale.max(1.0) {
        return None;
    }
    let x_axis = radial.scale(1.0 / radius);
    let axis = radial.cross(pa.sub(center)).normalized().ok()?;
    let y_axis = axis.cross(x_axis);
    let p_end = at(1.0).ok()?;
    let sweep = if p_end.sub(p0).length() <= 1e-9 * (1.0 + radius) {
        std::f64::consts::TAU
    } else {
        let closing = p_end.sub(center);
        let mut angle = closing.dot(y_axis).atan2(closing.dot(x_axis));
        if angle < 0.0 {
            angle += std::f64::consts::TAU;
        }
        angle
    };
    let rebuilt = make_arc(center, x_axis, y_axis, radius, 0.0, sweep).ok()?;
    curves_match(curve, &rebuilt, scale).then_some(CircularArc {
        center,
        axis,
        x_axis,
        y_axis,
        radius,
        sweep,
        spans: (curve.control_points.len() - 1) / 2,
    })
}

/// Emit LINE or CIRCLE for a recognized analytic edge curve (already oriented
/// start-to-end by `edge_subcurve`), or `None` for the B-spline fallback.
///
/// The second value describes the parameterization ISO 10303-42 gives the
/// entity that was written, because a pcurve on this edge has to share THAT
/// parameter — not the kernel's knot values (see `step/pcurve.rs`).
fn write_analytic_curve(
    writer: &mut StepWriter,
    curve: &NurbsCurve,
) -> Result<Option<(usize, EmittedCurve)>, String> {
    if curve.degree == 1
        && curve.control_points.len() == 2
        && curve
            .control_points
            .iter()
            .all(|control| (control.w - 1.0).abs() <= 1e-12)
    {
        let start = curve.control_points[0].point()?;
        let end = curve.control_points[1].point()?;
        let Ok(direction) = end.sub(start).normalized() else {
            return Ok(None);
        };
        let point = write_point(writer, start)?;
        let step_direction = write_direction(writer, direction)?;
        let vector = writer.add(format!(
            "VECTOR('',#{step_direction},{})",
            real(end.sub(start).length())?
        ));
        return Ok(Some((
            writer.add(format!("LINE('',#{point},#{vector})")),
            // The VECTOR carries the full chord length, so the STEP parameter
            // of this line is the fraction along the chord: C(s) = start +
            // s·(end − start) over [0, 1].
            EmittedCurve::Line { start, end },
        )));
    }
    if let Some(arc) = recognize_circular_arc(curve) {
        // ref_direction points at the edge's start vertex and the arc runs
        // counterclockwise about the axis, so the importer's vertex-trimmed
        // CCW rebuild reproduces the same directed curve with sense .T.
        let placement = write_placement(writer, arc.center, arc.axis, arc.x_axis)?;
        return Ok(Some((
            writer.add(format!("CIRCLE('',#{placement},{})", real(arc.radius)?)),
            EmittedCurve::Circle {
                center: arc.center,
                x_axis: arc.x_axis,
                y_axis: arc.y_axis,
                radius: arc.radius,
                sweep: arc.sweep,
                spans: arc.spans,
            },
        )));
    }
    Ok(None)
}

fn write_curve(writer: &mut StepWriter, curve: &NurbsCurve) -> Result<usize, String> {
    let points = curve
        .control_points
        .iter()
        .map(|control| write_point(writer, control.point()?))
        .collect::<Result<Vec<_>, _>>()?;
    write_bspline_curve(writer, curve, &points)
}

/// The B_SPLINE_CURVE_WITH_KNOTS entity (or its rational complex form) over
/// control points that have ALREADY been written — shared by the 3D edge
/// curves and by the 2D pcurves, whose CARTESIAN_POINTs carry two coordinates
/// instead of three but whose degree/knots/weights are written identically.
fn write_bspline_curve(
    writer: &mut StepWriter,
    curve: &NurbsCurve,
    points: &[usize],
) -> Result<usize, String> {
    let (knot_values, multiplicities) = knot_runs(&curve.knots);
    let multiplicities = format!(
        "({})",
        multiplicities
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",")
    );
    let knots = format!(
        "({})",
        knot_values
            .iter()
            .map(|value| real(*value))
            .collect::<Result<Vec<_>, _>>()?
            .join(",")
    );
    let rational = curve
        .control_points
        .iter()
        .any(|control| (control.w - 1.0).abs() > 1e-12);
    if !rational {
        return Ok(writer.add(format!(
            "B_SPLINE_CURVE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.F.,{multiplicities},{knots},.UNSPECIFIED.)",
            curve.degree,
            id_list(points),
        )));
    }
    let weights = format!(
        "({})",
        curve
            .control_points
            .iter()
            .map(|control| real(control.w))
            .collect::<Result<Vec<_>, _>>()?
            .join(",")
    );
    Ok(writer.add(format!(
        "(BOUNDED_CURVE()B_SPLINE_CURVE({},{},.UNSPECIFIED.,.F.,.F.)\
         B_SPLINE_CURVE_WITH_KNOTS({multiplicities},{knots},.UNSPECIFIED.)\
         CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE({weights})\
         REPRESENTATION_ITEM(''))",
        curve.degree,
        id_list(points),
    )))
}

fn write_point_2d(writer: &mut StepWriter, point: [f64; 2]) -> Result<usize, String> {
    Ok(writer.add(format!(
        "CARTESIAN_POINT('',({},{}))",
        real(point[0])?,
        real(point[1])?
    )))
}

fn write_direction_2d(writer: &mut StepWriter, direction: [f64; 2]) -> Result<usize, String> {
    Ok(writer.add(format!(
        "DIRECTION('',({},{}))",
        real(direction[0])?,
        real(direction[1])?
    )))
}

/// Write the 2D geometry of one pcurve.
///
/// The `VECTOR` of a 2D LINE carries its TRUE magnitude rather than being
/// normalized to 1: ISO 10303-42 parameterizes a line as pnt + u·(magnitude ×
/// orientation), so the magnitude is what makes the 2D parameter equal the 3D
/// entity's parameter exactly.  (OpenCASCADE normalizes every VECTOR it writes
/// and then loses that agreement wherever the two speeds differ — its own cone
/// ruling pcurves are off by cos(semi-angle) — so this is strictly the more
/// faithful of the two conventions, and identical wherever the speeds agree.)
fn write_pcurve_geometry(writer: &mut StepWriter, curve: &Pcurve2d) -> Result<usize, String> {
    match curve {
        Pcurve2d::Line { point, vector } => {
            let magnitude = vector[0].hypot(vector[1]);
            if magnitude <= 0.0 {
                return Err("export_step: degenerate 2D line pcurve".into());
            }
            let point_id = write_point_2d(writer, *point)?;
            let direction =
                write_direction_2d(writer, [vector[0] / magnitude, vector[1] / magnitude])?;
            let vector_id = writer.add(format!(
                "VECTOR('',#{direction},{})",
                real(magnitude)?
            ));
            Ok(writer.add(format!("LINE('',#{point_id},#{vector_id})")))
        }
        Pcurve2d::Circle {
            center,
            ref_direction,
            radius,
        } => {
            let center_id = write_point_2d(writer, *center)?;
            let direction = write_direction_2d(writer, *ref_direction)?;
            let placement = writer.add(format!(
                "AXIS2_PLACEMENT_2D('',#{center_id},#{direction})"
            ));
            Ok(writer.add(format!("CIRCLE('',#{placement},{})", real(*radius)?)))
        }
        Pcurve2d::Spline(spline) => {
            let points = spline
                .control_points
                .iter()
                .map(|control| {
                    let point = control.point()?;
                    write_point_2d(writer, [point.x, point.y])
                })
                .collect::<Result<Vec<_>, String>>()?;
            write_bspline_curve(writer, spline, &points)
        }
    }
}

/// `PCURVE('', surface, DEFINITIONAL_REPRESENTATION('', (2d curve), ctx))` —
/// the association of one 2D curve with the surface it parameterizes.
fn write_pcurve_entity(
    writer: &mut StepWriter,
    surface_id: usize,
    context_2d: usize,
    curve: &Pcurve2d,
) -> Result<usize, String> {
    let geometry = write_pcurve_geometry(writer, curve)?;
    let representation = writer.add(format!(
        "DEFINITIONAL_REPRESENTATION('',(#{geometry}),#{context_2d})"
    ));
    Ok(writer.add(format!("PCURVE('',#{surface_id},#{representation})")))
}

fn write_surface(writer: &mut StepWriter, surface: &NurbsSurface) -> Result<usize, String> {
    let rows = surface
        .control_points
        .iter()
        .map(|row| {
            row.iter()
                .map(|control| write_point(writer, control.point()?))
                .collect::<Result<Vec<_>, _>>()
                .map(|ids| id_list(&ids))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let grid = format!("({})", rows.join(","));
    let (u_values, u_multiplicities) = knot_runs(&surface.knots_u);
    let (v_values, v_multiplicities) = knot_runs(&surface.knots_v);
    let multiplicities = |values: &[usize]| {
        format!(
            "({})",
            values
                .iter()
                .map(usize::to_string)
                .collect::<Vec<_>>()
                .join(",")
        )
    };
    let knots = |values: &[f64]| -> Result<String, String> {
        Ok(format!(
            "({})",
            values
                .iter()
                .map(|value| real(*value))
                .collect::<Result<Vec<_>, _>>()?
                .join(",")
        ))
    };
    let u_mults = multiplicities(&u_multiplicities);
    let v_mults = multiplicities(&v_multiplicities);
    let u_knots = knots(&u_values)?;
    let v_knots = knots(&v_values)?;
    let rational = surface
        .control_points
        .iter()
        .flatten()
        .any(|control| (control.w - 1.0).abs() > 1e-12);
    if !rational {
        return Ok(writer.add(format!(
            "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{grid},.UNSPECIFIED.,.F.,.F.,.F.,\
             {u_mults},{v_mults},{u_knots},{v_knots},.UNSPECIFIED.)",
            surface.degree_u, surface.degree_v,
        )));
    }
    let weights = format!(
        "({})",
        surface
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|control| real(control.w))
                    .collect::<Result<Vec<_>, _>>()
                    .map(|values| format!("({})", values.join(",")))
            })
            .collect::<Result<Vec<_>, _>>()?
            .join(",")
    );
    Ok(writer.add(format!(
        "(BOUNDED_SURFACE()B_SPLINE_SURFACE({},{},{grid},.UNSPECIFIED.,.F.,.F.,.F.)\
         B_SPLINE_SURFACE_WITH_KNOTS({u_mults},{v_mults},{u_knots},{v_knots},.UNSPECIFIED.)\
         GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE({weights})\
         REPRESENTATION_ITEM('')SURFACE())",
        surface.degree_u, surface.degree_v,
    )))
}

fn write_length_unit(writer: &mut StepWriter, unit: &str) -> Result<usize, String> {
    let normalized = unit.to_lowercase();
    if normalized == "meter" || normalized == "metre" {
        return Ok(writer.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))"));
    }
    if normalized == "centimeter" || normalized == "centimetre" {
        return Ok(writer.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.CENTI.,.METRE.))"));
    }
    if matches!(normalized.as_str(), "micron" | "micrometer" | "micrometre") {
        return Ok(writer.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MICRO.,.METRE.))"));
    }
    if normalized == "inch" || normalized == "foot" {
        let metre = writer.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))");
        let (factor, name) = if normalized == "inch" {
            (0.0254, "INCH")
        } else {
            (0.3048, "FOOT")
        };
        let measure = writer.add(format!(
            "LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{metre})",
            real(factor)?
        ));
        return Ok(writer.add(format!(
            "(CONVERSION_BASED_UNIT('{name}',#{measure})LENGTH_UNIT()NAMED_UNIT(*))"
        )));
    }
    Ok(writer.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))"))
}

fn vertex_for(solid: &BrepSolid, id: u64) -> Result<&VertexRecord, String> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == id)
        .ok_or_else(|| format!("export_step: missing vertex {id}"))
}

fn edge_for(solid: &BrepSolid, id: u64) -> Result<&EdgeRecord, String> {
    solid
        .edges
        .iter()
        .find(|edge| edge.id == id)
        .ok_or_else(|| format!("export_step: missing edge {id}"))
}

fn surface_key(face: &FaceRecord) -> usize {
    face as *const FaceRecord as usize
}

/// One use of an edge by a face's loop.  Curve-on-surface geometry is written
/// per USE — a pcurve names the surface it lives on — so the writer needs the
/// full adjacency of an edge before it can emit that edge.
struct CoedgeUse<'a> {
    surface_key: usize,
    face: &'a FaceRecord,
    coedge: &'a CoedgeRecord,
}

/// The AP242 document plus what the writer measured while producing it.
///
/// The pcurve counters are the honest half of the verify-or-omit contract in
/// `step/pcurve.rs`: a pcurve is written only when the emitted 2D geometry was
/// proved to reproduce the emitted 3D curve, and every one that could not be
/// proved is COUNTED here rather than guessed into the file.  A reader
/// reprojects an omitted pcurve exactly as it must reproject every pcurve in
/// the files this exporter wrote before curve-on-surface geometry existed.
#[derive(Clone, Debug, Default)]
pub struct StepExportReport {
    /// The Part 21 text.
    pub text: String,
    /// `PCURVE` entities written — at most one per coedge use of a
    /// non-degenerate edge.
    pub pcurves_written: usize,
    /// Coedge uses left without a pcurve because no candidate 2D geometry
    /// verified inside the export band.
    pub pcurves_omitted: usize,
    /// Edges written as `SURFACE_CURVE` (their two uses sit on two surfaces).
    pub surface_curves: usize,
    /// Edges written as `SEAM_CURVE` (both uses on ONE surface — a periodic
    /// seam, the case an importer otherwise has to re-detect geometrically).
    pub seam_curves: usize,
    /// Edges that kept a bare 3D curve for `edge_geometry`, because no pcurve
    /// survived (or a seam lost one of the pair it is required to carry).
    pub bare_curves: usize,
    /// Collapsed face boundaries written as `VERTEX_LOOP` — cone apexes and
    /// sphere poles, which this writer used to drop entirely.
    pub vertex_loops: usize,
    /// Largest verified ‖S(c₂d(s)) − c₃d(s)‖ among the pcurves actually
    /// written, in model units.
    pub max_pcurve_deviation: f64,
    /// Largest deviation among the BEST candidate for each OMITTED pcurve —
    /// how far the closest miss was, so an omission can be diagnosed as "a
    /// candidate applied and missed the band by this much" rather than only
    /// counted. Zero when nothing was omitted.
    ///
    /// It is infinite when ANY omission had no candidate proposed at all, which
    /// then hides the finite misses behind it — the two omission classes share
    /// one counter. Splitting them is a follow-up; the max is the useful half,
    /// because it is the finite value that says whether widening the band would
    /// have helped.
    pub worst_omitted_deviation: f64,
    /// DISTINCT PMI reference names (`{body}`, `{body}:face`, `{body}@x,y,z`)
    /// that named no entity in the written file, so every annotation on them
    /// was skipped. Zero for a file whose PMI all resolved. The structured
    /// (assembly) lane is what can move a reference: a component's geometry is
    /// written into the PART's product, and a stale reference to a body that no
    /// longer exists is counted here rather than dropped in silence.
    pub pmi_unresolved_references: usize,
    /// PRODUCTs written. `1` for the flat lane; the structured lane writes one
    /// per distinct part plus one for the root document.
    pub products: usize,
    /// `NEXT_ASSEMBLY_USAGE_OCCURRENCE` edges written — the placed component
    /// instances. Zero for the flat lane.
    pub occurrences: usize,
    /// `STYLED_ITEM`s written for GEOMETRY colour — one per coloured body and
    /// per coloured face (`step/styles.rs`). Zero when the caller passed no
    /// colours, or when none of the names it passed named written geometry;
    /// the PMI block's own styled item is not counted here.
    pub styled_items: usize,
}

/// Serialize exact NURBS BREP topology as an AP242 STEP Part 21 document.
pub fn export_step(
    solids: &[BrepSolid],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    export_step_report(solids, name, unit, timestamp).map(|report| report.text)
}

/// [`export_step`] plus the pcurve-coverage measurements behind the file.
/// Every body is written under the part name; no PMI.
pub fn export_step_report(
    solids: &[BrepSolid],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<StepExportReport, String> {
    let named: Vec<(String, &BrepSolid)> = solids
        .iter()
        .map(|solid| (name.to_string(), solid))
        .collect();
    export_step_report_named(&named, name, unit, timestamp, None, None)
}

/// The full writer: each body carries its SCENE NAME (`MANIFOLD_SOLID_BREP`
/// name — and the key the PMI references resolve through), and an optional
/// PMI block is written as AP242 semantic representation + polyline
/// presentation + saved views ([`pmi`]).
/// Where an exported geometry entity lives: the product whose shape it helps
/// define, and the representation that carries it. PMI attaches a shape aspect
/// to the OWNING product, which in a structured (assembly) export is the part's
/// own product, not the root document's.
#[derive(Clone, Copy)]
pub(crate) struct StepItemOwner {
    pub product_shape: usize,
    pub representation: usize,
}

/// One product's geometry, written but not yet bound to a product definition.
/// The name lists are flat (not maps) because the same part can be reachable
/// under several occurrence paths, and each path registers its own alias.
#[derive(Default)]
pub(crate) struct ProductGeometry {
    /// Body name -> its `MANIFOLD_SOLID_BREP`, in emission order — one entry per
    /// SHELL the body wrote, so a body with voids appears once per shell. The
    /// name rides along because a coloured body is styled on this entity
    /// (`step/styles.rs`), and the representation's item list is the ids alone.
    pub solids: Vec<(String, usize)>,
    /// Face name -> its `ADVANCED_FACE`.
    pub faces: Vec<(String, usize)>,
    /// Edge name -> its `EDGE_CURVE`.
    pub edges: Vec<(String, usize)>,
    /// Body name -> that body's `VERTEX_POINT`s (LOCAL point, entity id).
    pub vertices: Vec<(String, Vec<(Vec3, usize)>)>,
}

/// The PMI reference maps, folded from every product's [`ProductGeometry`] once
/// its owner is known. A structured export registers each product's entities
/// under EVERY occurrence path that reaches it (`ACOMP3:`, `ACOMP5:ACOMP1:`),
/// which is exactly how the document names a component's geometry.
#[derive(Default)]
pub(crate) struct StepNameMaps {
    pub faces: HashMap<String, (usize, StepItemOwner)>,
    pub edges: HashMap<String, (usize, StepItemOwner)>,
    /// Body name -> its owner and its `VERTEX_POINT`s in ROOT (document) space,
    /// because that is the frame a `{body}@x,y,z` PMI reference is written in.
    pub vertices: HashMap<String, (StepItemOwner, Vec<(Vec3, usize)>)>,
}

impl StepNameMaps {
    /// Register one product's entities under an occurrence path: `prefix` is the
    /// chained component namespace (`""` at the root), `world` maps the
    /// product's local frame into root space.
    pub(crate) fn register(
        &mut self,
        geometry: &ProductGeometry,
        owner: StepItemOwner,
        prefix: &str,
        world: &Mat4,
    ) {
        for (name, id) in &geometry.faces {
            self.faces
                .entry(format!("{prefix}{name}"))
                .or_insert((*id, owner));
        }
        for (name, id) in &geometry.edges {
            self.edges
                .entry(format!("{prefix}{name}"))
                .or_insert((*id, owner));
        }
        for (name, points) in &geometry.vertices {
            let placed = points
                .iter()
                .map(|(point, id)| (transform_point(world, *point), *id))
                .collect();
            self.vertices
                .entry(format!("{prefix}{name}"))
                .or_insert((owner, placed));
        }
    }
}

/// The file-wide entities every product's geometry is expressed against. One
/// set per FILE, shared by every product a structured export writes.
pub(crate) struct StepFileContexts {
    pub product_context: usize,
    pub definition_context: usize,
    pub geometry_context: usize,
    pub parametric_context: usize,
    pub length_unit: usize,
    pub angle_unit: usize,
    /// The IDENTITY `AXIS2_PLACEMENT_3D` — the first item of every shape
    /// representation, and `item_1` of every assembly placement's
    /// `ITEM_DEFINED_TRANSFORMATION` (the child frame's own origin).
    pub axis: usize,
}

/// Write the header entities every AP242 body in this file shares.
pub(crate) fn write_file_contexts(
    writer: &mut StepWriter,
    unit: &str,
) -> Result<StepFileContexts, String> {
    let application = writer.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    writer.add(format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2014,#{application})"
    ));
    let product_context = writer.add(format!("PRODUCT_CONTEXT('',#{application},'mechanical')"));
    let definition_context = writer.add(format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{application},'design')"
    ));
    let length_unit = write_length_unit(writer, unit)?;
    let angle_unit = writer.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let solid_angle_unit = writer.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let uncertainty = writer.add(format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-6),#{length_unit},'distance_accuracy_value','')"
    ));
    let geometry_context = writer.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)\
         GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty}))\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length_unit},#{angle_unit},#{solid_angle_unit}))\
         REPRESENTATION_CONTEXT('',''))"
    ));
    // ONE parametric context for every DEFINITIONAL_REPRESENTATION in the file.
    // It carries no unit assignment, so the importer's file-scale search
    // (`derive_length_scale_mm`, which keys on GLOBAL_UNIT_ASSIGNED_CONTEXT /
    // LENGTH_UNIT) cannot mistake it for the geometric context.
    let parametric_context = writer.add(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(2)\
         PARAMETRIC_REPRESENTATION_CONTEXT()\
         REPRESENTATION_CONTEXT('2D SPACE',''))",
    );
    let origin = write_point(writer, Vec3::default())?;
    let direction_z = writer.add("DIRECTION('',(0.,0.,1.))");
    let direction_x = writer.add("DIRECTION('',(1.,0.,0.))");
    let axis = writer.add(format!(
        "AXIS2_PLACEMENT_3D('',#{origin},#{direction_z},#{direction_x})"
    ));
    Ok(StepFileContexts {
        product_context,
        definition_context,
        geometry_context,
        parametric_context,
        length_unit,
        angle_unit,
        axis,
    })
}

/// The product-definition chain of ONE product: what a `NEXT_ASSEMBLY_USAGE_
/// OCCURRENCE` references and what a `SHAPE_DEFINITION_REPRESENTATION` binds to
/// its geometry.
pub(crate) struct ProductIds {
    pub definition: usize,
    pub product_shape: usize,
}

/// PRODUCT -> PRODUCT_DEFINITION -> PRODUCT_DEFINITION_SHAPE for one product.
/// `id` is the vendor part number (`PRODUCT.id`); the name is used when empty.
/// `category` is the `PRODUCT_RELATED_PRODUCT_CATEGORY` name — `'part'` for a
/// leaf, `'assembly'` for a product that places others, which is the
/// distinction a receiving system reads to build its own structure tree.
pub(crate) fn write_product(
    writer: &mut StepWriter,
    contexts: &StepFileContexts,
    name: &str,
    id: &str,
    category: &str,
) -> ProductIds {
    let safe_name = step_string(name);
    let safe_id = step_string(if id.is_empty() { name } else { id });
    let product_context = contexts.product_context;
    let definition_context = contexts.definition_context;
    let product = writer.add(format!(
        "PRODUCT('{safe_id}','{safe_name}','',(#{product_context}))"
    ));
    writer.add(format!(
        "PRODUCT_RELATED_PRODUCT_CATEGORY('{}','',(#{product}))",
        step_string(category)
    ));
    let formation = writer.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let definition = writer.add(format!(
        "PRODUCT_DEFINITION('design','',#{formation},#{definition_context})"
    ));
    let product_shape = writer.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    ProductIds {
        definition,
        product_shape,
    }
}

/// Validate every body the way the export gate requires, then write each one's
/// exact topology as a `MANIFOLD_SOLID_BREP` named by its scene name.
///
/// This is the whole geometry writer, factored out of the single-product lane so
/// the structured (assembly) lane can call it once PER PRODUCT into the same
/// file. It appends to `writer` and reads only file-wide entities, so products
/// share surfaces with nothing and cannot collide.
pub(crate) fn write_product_geometry(
    writer: &mut StepWriter,
    contexts: &StepFileContexts,
    bodies: &[(String, &BrepSolid)],
    report: &mut StepExportReport,
) -> Result<ProductGeometry, String> {
    for (_, solid) in bodies {
        let policy = KernelTolerances::for_solid(solid, 1e-7);
        let issues = solid.validate_with_tolerances(&KernelTolerances {
            pcurve_consistency: policy.export_knit,
            ..policy
        });
        if !issues.is_empty() {
            return Err(format!("export_step: invalid solid: {issues:?}"));
        }
    }
    let parametric_context = contexts.parametric_context;
    let mut written = ProductGeometry::default();
    for (solid_name, solid) in bodies {
        let solid = *solid;
        let solid_name = solid_name.as_str();
        let band = KernelTolerances::for_solid(solid, 1e-7).export_knit;
        let mut vertex_ids = HashMap::<u64, usize>::default();
        let mut edge_ids = HashMap::<u64, usize>::default();
        let mut surfaces = HashMap::<usize, (usize, bool, EmittedSurface)>::default();
        for shell in &solid.shells {
            // Pass 1 — SURFACES. A pcurve references the surface entity it
            // parameterizes, so every surface id has to exist before the first
            // edge is written. (Before curve-on-surface geometry the writer
            // could emit surfaces after the loops; it no longer can.)
            for face in &shell.faces {
                let key = surface_key(face);
                if surfaces.contains_key(&key) {
                    continue;
                }
                let entry = match write_analytic_surface(writer, &face.surface)? {
                    Some(triple) => triple,
                    None => (
                        write_surface(writer, &face.surface)?,
                        false,
                        EmittedSurface::Spline,
                    ),
                };
                surfaces.insert(key, entry);
            }

            // Pass 2 — ADJACENCY. Which faces use each edge, in first-encounter
            // order so the emitted entity numbering stays deterministic.
            let mut edge_uses = HashMap::<u64, Vec<CoedgeUse>>::default();
            let mut edge_order: Vec<u64> = Vec::new();
            for face in &shell.faces {
                for loop_record in &face.loops {
                    for coedge in &loop_record.coedges {
                        let edge = edge_for(solid, coedge.edge_id)?;
                        if edge.degenerate {
                            continue;
                        }
                        let uses = edge_uses.entry(edge.id).or_default();
                        if uses.is_empty() {
                            edge_order.push(edge.id);
                        }
                        uses.push(CoedgeUse {
                            surface_key: surface_key(face),
                            face,
                            coedge,
                        });
                    }
                }
            }

            // Pass 3 — EDGES with their curve-on-surface geometry.
            for edge_id in &edge_order {
                if edge_ids.contains_key(edge_id) {
                    continue;
                }
                let edge = edge_for(solid, *edge_id)?;
                let subcurve = edge_subcurve(edge)?;
                let (curve, emitted_curve) = match write_analytic_curve(writer, &subcurve)? {
                    Some(pair) => pair,
                    None => (
                        write_curve(writer, &subcurve)?,
                        EmittedCurve::Spline { curve: subcurve },
                    ),
                };
                let uses = &edge_uses[edge_id];
                // Two uses on ONE surface is a periodic seam: the two halves of
                // the edge sit on opposite domain boundaries and the reader is
                // told so explicitly instead of having to rediscover it.
                let seam = uses.len() == 2 && uses[0].surface_key == uses[1].surface_key;
                let mut pcurves: Vec<(usize, Pcurve2d)> = Vec::new();
                let mut omitted = 0usize;
                if uses.len() == 2 {
                    // Slot order for a seam: the FORWARD-oriented coedge first,
                    // the reversed one second — the pairing OpenCASCADE writes
                    // and reads (BRep_CurveOnClosedSurface's PCurve1/PCurve2).
                    let mut ordered: Vec<&CoedgeUse> = uses.iter().collect();
                    if seam && !ordered[0].coedge.forward {
                        ordered.swap(0, 1);
                    }
                    for coedge_use in ordered {
                        // The stored pcurve runs in LOOP direction; the edge's
                        // geometry runs in EDGE direction. Fraction-synchronised
                        // reversal is exact, so a reversed coedge's pcurve is
                        // simply flipped before it is expressed.
                        let oriented = if coedge_use.coedge.forward {
                            coedge_use.coedge.pcurve.clone()
                        } else {
                            coedge_use.coedge.pcurve.reversed()?
                        };
                        let (surface_id, _, emitted_surface) = &surfaces[&coedge_use.surface_key];
                        let outcome = build_pcurve(
                            &coedge_use.face.surface,
                            emitted_surface,
                            &emitted_curve,
                            &oriented,
                            band,
                        )?;
                        match outcome.curve {
                            Some(curve_2d) => {
                                report.max_pcurve_deviation =
                                    report.max_pcurve_deviation.max(outcome.deviation);
                                pcurves.push((*surface_id, curve_2d));
                            }
                            None => {
                                omitted += 1;
                                if outcome.deviation.is_finite() {
                                    report.worst_omitted_deviation =
                                        report.worst_omitted_deviation.max(outcome.deviation);
                                } else {
                                    report.worst_omitted_deviation = f64::INFINITY;
                                }
                            }
                        }
                    }
                }
                // A SEAM_CURVE is required to carry BOTH pcurves on the one
                // surface, so a seam that lost one falls all the way back to a
                // bare 3D curve rather than to a half-described seam.
                if seam && pcurves.len() != 2 {
                    omitted += pcurves.len();
                    pcurves.clear();
                }
                report.pcurves_omitted += omitted;
                report.pcurves_written += pcurves.len();
                let geometry = if pcurves.is_empty() {
                    report.bare_curves += 1;
                    curve
                } else {
                    let ids = pcurves
                        .iter()
                        .map(|(surface_id, curve_2d)| {
                            write_pcurve_entity(
                                writer,
                                *surface_id,
                                parametric_context,
                                curve_2d,
                            )
                        })
                        .collect::<Result<Vec<_>, String>>()?;
                    let keyword = if seam {
                        report.seam_curves += 1;
                        "SEAM_CURVE"
                    } else {
                        report.surface_curves += 1;
                        "SURFACE_CURVE"
                    };
                    // master_representation is .CURVE_3D.: our 3D curves are
                    // the exact authority the whole kernel and the export gate
                    // treat them as, and the pcurves above are verified AGAINST
                    // them. Promoting an approximation to master would be the
                    // one claim this writer must not make.
                    writer.add(format!(
                        "{keyword}('',#{curve},{},.CURVE_3D.)",
                        id_list(&ids)
                    ))
                };
                let start = vertex_step_id(writer, &mut vertex_ids, solid, edge.start_vertex_id)?;
                let end = vertex_step_id(writer, &mut vertex_ids, solid, edge.end_vertex_id)?;
                let step_id =
                    writer.add(format!("EDGE_CURVE('',#{start},#{end},#{geometry},.T.)"));
                edge_ids.insert(edge.id, step_id);
                if let Some(edge_name) = edge.name.as_deref() {
                    written.edges.push((edge_name.to_string(), step_id));
                }
            }

            // Pass 4 — LOOPS and FACES over the ids the passes above fixed.
            let mut face_ids = Vec::new();
            for face in &shell.faces {
                let mut bound_ids = Vec::new();
                for (loop_index, loop_record) in face.loops.iter().enumerate() {
                    let mut oriented_edges = Vec::new();
                    for coedge in &loop_record.coedges {
                        let edge = edge_for(solid, coedge.edge_id)?;
                        if edge.degenerate {
                            continue;
                        }
                        let edge_id = *edge_ids
                            .get(&edge.id)
                            .ok_or_else(|| format!("export_step: unwritten edge {}", edge.id))?;
                        let orientation = if coedge.forward { ".T." } else { ".F." };
                        oriented_edges.push(
                            writer.add(format!("ORIENTED_EDGE('',*,*,#{edge_id},{orientation})")),
                        );
                    }
                    let kind = if loop_index == 0 {
                        "FACE_OUTER_BOUND"
                    } else {
                        "FACE_BOUND"
                    };
                    if oriented_edges.is_empty() {
                        // A bound left empty by the degenerate-edge skip is a
                        // COLLAPSED boundary — a cone apex, a sphere pole —
                        // and AP242 spells that `VERTEX_LOOP`, not nothing.
                        // Dropping it (what this writer did before) deletes the
                        // apex from the face's parameter domain, so an imported
                        // pointed cone lost its apex bound on re-export and
                        // every reader had to re-synthesise it from the
                        // surface's own degeneracy. A MIXED loop keeps the
                        // skip: that is OCCT's own write convention, both our
                        // importer and OCC's ShapeFix rebuild those, and the
                        // advanced-face conformance class permits vertex loops
                        // but not zero-length edge curves.
                        let Some(coedge) = loop_record.coedges.first() else {
                            continue;
                        };
                        let collapsed = edge_for(solid, coedge.edge_id)?;
                        let vertex = vertex_step_id(
                            writer,
                            &mut vertex_ids,
                            solid,
                            collapsed.start_vertex_id,
                        )?;
                        let vertex_loop = writer.add(format!("VERTEX_LOOP('',#{vertex})"));
                        report.vertex_loops += 1;
                        bound_ids.push(writer.add(format!("{kind}('',#{vertex_loop},.T.)")));
                        continue;
                    }
                    let edge_loop =
                        writer.add(format!("EDGE_LOOP('',{})", id_list(&oriented_edges)));
                    bound_ids.push(writer.add(format!("{kind}('',#{edge_loop},.T.)")));
                }
                let (surface, flipped, _) = surfaces[&surface_key(face)];
                // `flipped` analytic entities are written with the reverse of
                // the stored NURBS orientation, so invert the flag to keep the
                // face normal identical through the round trip.
                let sense = if face.same_sense != flipped {
                    ".T."
                } else {
                    ".F."
                };
                let face_step_id = writer.add(format!(
                    "ADVANCED_FACE('',{},#{surface},{sense})",
                    id_list(&bound_ids)
                ));
                if let Some(face_name) = face.name.as_deref() {
                    written.faces.push((face_name.to_string(), face_step_id));
                }
                face_ids.push(face_step_id);
            }
            let closed_shell = writer.add(format!("CLOSED_SHELL('',{})", id_list(&face_ids)));
            let body_id = writer.add(format!(
                "MANIFOLD_SOLID_BREP('{}',#{closed_shell})",
                step_string(solid_name)
            ));
            written.solids.push((solid_name.to_string(), body_id));
        }
        // The vertices this solid wrote, for `{solid}@x,y,z` PMI references.
        let mut points: Vec<(Vec3, usize)> = Vec::with_capacity(vertex_ids.len());
        for (vertex_id, step_id) in &vertex_ids {
            points.push((vertex_for(solid, *vertex_id)?.point, *step_id));
        }
        written.vertices.push((solid_name.to_string(), points));
    }
    Ok(written)
}

/// Wrap the written entities in the Part 21 envelope and run the emitted-text
/// audits — the last step of every export lane.
pub(crate) fn finish_step_file(
    writer: StepWriter,
    name: &str,
    timestamp: &str,
    report: &mut StepExportReport,
) -> Result<(), String> {
    let safe_name = step_string(name);
    let safe_timestamp = step_string(timestamp);
    let output = [
        "ISO-10303-21;".to_string(),
        "HEADER;".to_string(),
        "FILE_DESCRIPTION((''),'2;1');".to_string(),
        format!(
            "FILE_NAME('{safe_name}.step','{safe_timestamp}',(''),(''),'brep-kernel-rs','brep-kernel-rs','');"
        ),
        "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'));".to_string(),
        "ENDSEC;".to_string(),
        "DATA;".to_string(),
        writer.data(),
        "ENDSEC;".to_string(),
        "END-ISO-10303-21;".to_string(),
        String::new(),
    ]
    .join("\n");
    let manifold_issues = audit_step_manifold(&output);
    if !manifold_issues.is_empty() {
        return Err(format!(
            "export_step: emitted AP242 manifold audit failed: {}",
            manifold_issues.join("; ")
        ));
    }
    let pcurve_issues = audit_step_pcurves(&output);
    if !pcurve_issues.is_empty() {
        return Err(format!(
            "export_step: emitted AP242 pcurve audit failed: {}",
            pcurve_issues.join("; ")
        ));
    }
    report.text = output;
    Ok(())
}

/// The full single-product writer: each body carries its SCENE NAME
/// (`MANIFOLD_SOLID_BREP` name — and the key the PMI references resolve
/// through), and an optional PMI block is written as AP242 semantic
/// representation + polyline presentation + saved views ([`pmi`]).
///
/// This is the FLAT lane: one `PRODUCT`, every body in it, no assembly
/// structure. A document with components goes through
/// [`assembly::export_step_assembly_report`] instead.
pub fn export_step_report_named(
    solids: &[(String, &BrepSolid)],
    name: &str,
    unit: &str,
    timestamp: &str,
    pmi: Option<&StepPmi<'_>>,
    colors: Option<&StepColors>,
) -> Result<StepExportReport, String> {
    if solids.is_empty() {
        return Err("export_step: at least one solid is required".into());
    }
    let mut report = StepExportReport {
        products: 1,
        ..StepExportReport::default()
    };
    let mut writer = StepWriter::default();
    let contexts = write_file_contexts(&mut writer, unit)?;
    let geometry = write_product_geometry(&mut writer, &contexts, solids, &mut report)?;
    let mut items = vec![contexts.axis];
    items.extend(geometry.solids.iter().map(|(_, id)| *id));
    let geometry_context = contexts.geometry_context;
    let representation = writer.add(format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('',{},#{geometry_context})",
        id_list(&items)
    ));
    let product = write_product(&mut writer, &contexts, name, "", "part");
    let product_shape = product.product_shape;
    writer.add(format!(
        "SHAPE_DEFINITION_REPRESENTATION(#{product_shape},#{representation})"
    ));
    // Geometry colour, before the PMI block so a file's presentation styles read
    // in geometry order. The flat lane's bodies carry their document names with
    // no component namespace, so the ONE prefix to resolve through is the empty
    // one.
    if let Some(colors) = colors.filter(|colors| !colors.is_empty()) {
        let items = styles::styled_items(&geometry, colors, &[String::new()]);
        report.styled_items = styles::write_geometry_styles(&mut writer, geometry_context, &items)?;
    }
    if let Some(pmi) = pmi {
        let mut names = StepNameMaps::default();
        names.register(
            &geometry,
            StepItemOwner {
                product_shape,
                representation,
            },
            "",
            &MAT4_IDENTITY,
        );
        let context = pmi::StepContext {
            product_shape,
            representation,
            geometry_context,
            length_unit: contexts.length_unit,
            angle_unit: contexts.angle_unit,
            faces: &names.faces,
            edges: &names.edges,
            vertices: &names.vertices,
        };
        report.pmi_unresolved_references = pmi::write_pmi(&mut writer, &context, pmi)?;
    }
    finish_step_file(writer, name, timestamp, &mut report)?;
    Ok(report)
}

/// VERTEX_POINT for a kernel vertex, written once per solid.
fn vertex_step_id(
    writer: &mut StepWriter,
    vertex_ids: &mut HashMap<u64, usize>,
    solid: &BrepSolid,
    id: u64,
) -> Result<usize, String> {
    if let Some(step_id) = vertex_ids.get(&id) {
        return Ok(*step_id);
    }
    let point = write_point(writer, vertex_for(solid, id)?.point)?;
    let step_id = writer.add(format!("VERTEX_POINT('',#{point})"));
    vertex_ids.insert(id, step_id);
    Ok(step_id)
}

/// Entity id -> body, for the serialized-text audits.  One entity per line is
/// this writer's own invariant, so the "parse" is a split.
fn step_entity_bodies(step: &str) -> HashMap<u64, &str> {
    step.lines()
        .filter_map(|line| {
            let rest = line.strip_prefix('#')?;
            let (digits, body) = rest.split_once('=')?;
            Some((
                digits.parse::<u64>().ok()?,
                body.trim_end().trim_end_matches(';'),
            ))
        })
        .collect()
}

/// Every `#N` reference in an entity body, in order.  For the entities audited
/// below that order IS the attribute order (`SURFACE_CURVE` puts its 3D curve
/// first and its pcurves after; `PCURVE` puts its surface first).
fn step_entity_refs(body: &str) -> Vec<u64> {
    let mut refs = Vec::new();
    let bytes = body.as_bytes();
    let mut index = 0;
    while index < bytes.len() {
        if bytes[index] == b'#' {
            let start = index + 1;
            let mut end = start;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
            }
            if end > start {
                if let Ok(id) = body[start..end].parse::<u64>() {
                    refs.push(id);
                }
            }
            index = end;
        } else {
            index += 1;
        }
    }
    refs
}

/// Audit the curve-on-surface half of the emitted graph, on the serialized
/// text, for the same reason `audit_step_manifold` exists: valid in-memory
/// intent is not proof of a correctly written file.
///
/// An `EDGE_CURVE` whose geometry is a bare 3D curve is NOT an issue — that is
/// the counted, deliberate outcome of the verify-or-omit gate. What is an
/// issue is a malformed bundle: a `SURFACE_CURVE` with no or more than two
/// associated geometries, a `SEAM_CURVE` that does not carry exactly two, a
/// `SEAM_CURVE` whose two pcurves name DIFFERENT surfaces (it is by definition
/// one surface's seam), a `SURFACE_CURVE` whose two pcurves name the SAME
/// surface (that is a seam, and must say so), or a `PCURVE` that does not
/// point at both a surface and a `DEFINITIONAL_REPRESENTATION`.
pub fn audit_step_pcurves(step: &str) -> Vec<String> {
    let bodies = step_entity_bodies(step);
    let mut issues = Vec::new();
    for (id, body) in &bodies {
        if !body.starts_with("EDGE_CURVE(") {
            continue;
        }
        let refs = step_entity_refs(body);
        let Some(geometry) = refs.get(2) else {
            issues.push(format!("EDGE_CURVE #{id} has no edge_geometry"));
            continue;
        };
        let Some(wrapper) = bodies.get(geometry) else {
            issues.push(format!("EDGE_CURVE #{id} references missing #{geometry}"));
            continue;
        };
        let seam = wrapper.starts_with("SEAM_CURVE(");
        if !seam && !wrapper.starts_with("SURFACE_CURVE(") {
            continue;
        }
        let wrapper_refs = step_entity_refs(wrapper);
        let pcurves = &wrapper_refs[wrapper_refs.len().min(1)..];
        if pcurves.is_empty() || pcurves.len() > 2 || (seam && pcurves.len() != 2) {
            issues.push(format!(
                "#{geometry} carries {} associated geometries",
                pcurves.len()
            ));
            continue;
        }
        let mut surfaces = Vec::new();
        for pcurve in pcurves {
            let Some(pcurve_body) = bodies.get(pcurve) else {
                issues.push(format!("#{geometry} references missing #{pcurve}"));
                continue;
            };
            if !pcurve_body.starts_with("PCURVE(") {
                issues.push(format!("#{geometry} associate #{pcurve} is not a PCURVE"));
                continue;
            }
            let pcurve_refs = step_entity_refs(pcurve_body);
            let representation = pcurve_refs.get(1).and_then(|id| bodies.get(id));
            if !representation.is_some_and(|body| body.starts_with("DEFINITIONAL_REPRESENTATION("))
            {
                issues.push(format!(
                    "PCURVE #{pcurve} has no DEFINITIONAL_REPRESENTATION"
                ));
            }
            if let Some(surface) = pcurve_refs.first() {
                surfaces.push(*surface);
            }
        }
        if surfaces.len() == 2 && (surfaces[0] == surfaces[1]) != seam {
            issues.push(format!(
                "#{geometry} pcurves name {} surface(s) but it is a {}",
                if surfaces[0] == surfaces[1] { 1 } else { 2 },
                if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" }
            ));
        }
    }
    issues.sort();
    issues
}

/// Audit the serialized entity graph rather than assuming that valid
/// in-memory topology was necessarily written correctly.  Every EDGE_CURVE
/// in a closed shell must have exactly two ORIENTED_EDGE users with opposite
/// senses.
pub fn audit_step_manifold(step: &str) -> Vec<String> {
    let marker = "ORIENTED_EDGE('',*,*,#";
    let mut uses = HashMap::<u64, Vec<bool>>::default();
    for line in step.lines() {
        let Some(offset) = line.find(marker) else {
            continue;
        };
        let rest = &line[offset + marker.len()..];
        let digits = rest
            .chars()
            .take_while(|character| character.is_ascii_digit())
            .collect::<String>();
        let Ok(edge_id) = digits.parse::<u64>() else {
            continue;
        };
        let suffix = &rest[digits.len()..];
        let sense = suffix.starts_with(",.T.");
        uses.entry(edge_id).or_default().push(sense);
    }
    let mut issues = uses
        .into_iter()
        .filter_map(|(edge, senses)| {
            (senses.len() != 2 || senses[0] == senses[1]).then(|| {
                format!(
                    "EDGE_CURVE #{edge} has {} uses with senses {:?}",
                    senses.len(),
                    senses
                )
            })
        })
        .collect::<Vec<_>>();
    issues.sort();
    issues
}

