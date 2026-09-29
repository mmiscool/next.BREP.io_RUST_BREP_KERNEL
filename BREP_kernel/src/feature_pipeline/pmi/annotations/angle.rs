//! Angle dimension (`ANG`): the angle between two planar faces / planes /
//! straight edges, folded to acute, obtuse or reflex.

use super::{boolean_field, decimals_of, dimension_fields, id_field, options_field, params, reference_field, schema_entry, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, class_of, direction_of, resolve_reference};
use crate::feature_pipeline::pmi::{format_dimension, PmiAnnotation, PmiContext, PmiGeometry, Resolved, ToleranceBlock};
use crate::feature_pipeline::SelectionProbe;
use crate::{SelectionGeometry, Vec3};

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "angle",
    short_name: "ANG",
    icon: "\u{2220}",
    long_name: "\u{2220} Angle dimension",
    label: "Angle dimension",
    applicable,
    schema,
    resolve,
};

/// Exactly two of faces / edges / planes.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges + probe.planes == 2
        && probe.vertices == 0
        && probe.solids == 0
        && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    let mut fields = params(vec![
        ("id", id_field()),
        (
            "targets",
            reference_field(
                "Targets",
                &["FACE", "EDGE", "PLANE"],
                true,
                2,
                2,
                "Two planar faces, planes or straight edges",
            ),
        ),
        (
            "angleType",
            options_field(
                "Angle",
                &["acute", "obtuse", "reflex"],
                "acute",
                "Which of the four angles between the elements to dimension",
            ),
        ),
        (
            "reverseElementOrder",
            boolean_field("Reverse order", false, "Sweep the arc from the second element to the first"),
        ),
    ]);
    dimension_fields(&mut fields, 1);
    schema_entry(&DEF, fields)
}

/// The arc plane direction of an element: a line's direction; for a plane,
/// the in-plane direction perpendicular to the dihedral axis (`normal ×
/// axis`), so the arc lies between the two faces as drawn on a drawing.
fn arc_direction(geometry: &SelectionGeometry, axis: Vec3) -> Vec3 {
    match geometry {
        SelectionGeometry::Plane { normal, .. } => normal.cross(axis),
        other => direction_of(other).unwrap_or(Vec3::new(1.0, 0.0, 0.0)),
    }
}

/// The arc vertex: for two lines their closest points' midpoint; for two
/// planes a point on the intersection line nearest the origins' midpoint;
/// mixed: the line's crossing of the plane, else its foot.
fn vertex_of(first: &SelectionGeometry, second: &SelectionGeometry, axis: Vec3) -> Vec3 {
    let origin = |g: &SelectionGeometry| g.representative_point();
    match (first, second) {
        (
            SelectionGeometry::Plane {
                origin: oa,
                normal: na,
            },
            SelectionGeometry::Plane {
                origin: ob,
                normal: nb,
            },
        ) => {
            // Solve for a point on both planes closest to the origins' midpoint.
            let mid = oa.add(*ob).scale(0.5);
            let na = *na;
            let nb = *nb;
            let da = oa.sub(mid).dot(na);
            let db = ob.sub(mid).dot(nb);
            let nn = na.dot(nb);
            let det = 1.0 - nn * nn;
            if det.abs() < 1e-12 {
                return mid;
            }
            let ka = (da - db * nn) / det;
            let kb = (db - da * nn) / det;
            let point = mid.add(na.scale(ka)).add(nb.scale(kb));
            // Slide along the axis to the midpoint's foot.
            let t = mid.sub(point).dot(axis);
            point.add(axis.scale(t))
        }
        (SelectionGeometry::Plane { origin: po, normal }, line) | (line, SelectionGeometry::Plane { origin: po, normal }) => {
            let lo = origin(line);
            let ld = direction_of(line).unwrap_or(Vec3::new(1.0, 0.0, 0.0));
            let denom = ld.dot(*normal);
            if denom.abs() > 1e-9 {
                let t = po.sub(lo).dot(*normal) / denom;
                lo.add(ld.scale(t))
            } else {
                lo
            }
        }
        (a, b) => {
            let (oa, da) = (origin(a), direction_of(a).unwrap_or(Vec3::new(1.0, 0.0, 0.0)));
            let (ob, db) = (origin(b), direction_of(b).unwrap_or(Vec3::new(0.0, 1.0, 0.0)));
            let w = oa.sub(ob);
            let aa = da.dot(da);
            let bb = da.dot(db);
            let cc = db.dot(db);
            let dd = da.dot(w);
            let ee = db.dot(w);
            let denom = aa * cc - bb * bb;
            if denom.abs() < 1e-12 {
                return oa.add(ob).scale(0.5);
            }
            let s = (bb * ee - cc * dd) / denom;
            let t = (aa * ee - bb * dd) / denom;
            oa.add(da.scale(s)).add(ob.add(db.scale(t))).scale(0.5)
        }
    }
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("targets");
    let [first, second] = targets.as_slice() else {
        return Err("select two planar faces, planes or straight edges".into());
    };
    let mut ga = resolve_reference(context.scene, first)?;
    let mut gb = resolve_reference(context.scene, second)?;
    if annotation.flag("reverseElementOrder") {
        std::mem::swap(&mut ga, &mut gb);
    }
    for (name, geometry) in [(first, &ga), (second, &gb)] {
        if direction_of(geometry).is_none() || matches!(geometry, SelectionGeometry::Circle { .. } | SelectionGeometry::Axis { .. }) {
            return Err(format!(
                "'{name}' is a {} — an angle needs planar faces, planes or straight edges",
                class_of(geometry)
            ));
        }
    }
    let na = direction_of(&ga).expect("checked");
    let nb = direction_of(&gb).expect("checked");
    let axis = na.cross(nb);
    if axis.length() < 1e-7 {
        return Err("the two elements are parallel — there is no angle to dimension".into());
    }
    let axis = axis.normalized().map_err(|e| e.to_string())?;
    let vertex = vertex_of(&ga, &gb, axis);
    let u = arc_direction(&ga, axis).normalized().map_err(|e| e.to_string())?;
    let v = arc_direction(&gb, axis).normalized().map_err(|e| e.to_string())?;
    // The signed sweep from u to v about the axis, in (0, 360).
    let mut sweep = u.cross(v).dot(axis).atan2(u.dot(v)).to_degrees();
    if sweep < 0.0 {
        sweep += 360.0;
    }
    let interior = if sweep > 180.0 { 360.0 - sweep } else { sweep };
    let (dir_a, dir_b, degrees) = match annotation.text("angleType").to_ascii_lowercase().as_str() {
        "obtuse" => {
            if interior >= 90.0 {
                (u, v, sweep.min(360.0 - sweep))
            } else {
                (u, v.scale(-1.0), 180.0 - interior)
            }
        }
        "reflex" => {
            let acute = if interior <= 90.0 { (u, v) } else { (u, v.scale(-1.0)) };
            let inner = if interior <= 90.0 { interior } else { 180.0 - interior };
            (acute.0, acute.1, 360.0 - (180.0 - inner))
        }
        _ => {
            if interior <= 90.0 {
                (u, v, interior)
            } else {
                (u, v.scale(-1.0), 180.0 - interior)
            }
        }
    };
    // Re-orient so `degrees` is the positive sweep from dir_a to dir_b about axis.
    let signed = dir_a.cross(dir_b).dot(axis).atan2(dir_a.dot(dir_b)).to_degrees();
    let axis = if signed < 0.0 { axis.scale(-1.0) } else { axis };
    let decimals = decimals_of(annotation, context, 1.0);
    let tolerance = ToleranceBlock::read(annotation, context.env)?;
    let text = format_dimension(degrees, decimals, &tolerance, annotation.flag("isReference"), "", "\u{00B0}");
    // Default label: along the bisector of the drawn arc.
    let half = (degrees * 0.5).to_radians();
    let bisector = rotate(dir_a, axis, half);
    let scale = ga.representative_point().sub(vertex).length().max(gb.representative_point().sub(vertex).length()).max(1.0);
    let default_label = vertex.add(bisector.scale(scale * 0.6));
    Ok(Resolved {
        text,
        value: Some(degrees),
        unit: "deg",
        references: targets,
        geometry: PmiGeometry::Angular {
            vertex: a3(vertex),
            dir_a: a3(dir_a),
            dir_b: a3(dir_b),
            axis: a3(axis),
            degrees,
        },
        default_label: a3(default_label),
    })
}

/// Rodrigues rotation of `v` about unit `axis` by `angle` radians.
pub fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v.scale(c)
        .add(axis.cross(v).scale(s))
        .add(axis.scale(axis.dot(v) * (1.0 - c)))
}
