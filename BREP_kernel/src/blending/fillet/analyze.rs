use super::*;
use crate::projection::SurfaceProjection;
use crate::{parameter_point_in_face, project_point_to_surface, PointClass, PolygonClass, SolidClassifier, Vec2};

pub(super) fn edge_and_faces<'a>(
    solid: &'a BrepSolid,
    edge_id: u64,
) -> Result<(&'a EdgeRecord, Vec<&'a FaceRecord>), KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "edge_id",
                format!("fillet: edge {edge_id} not found"),
            )
        })?;
    let mut faces = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .any(|coedge| coedge.edge_id == edge_id)
                && !faces.iter().any(|known: &&FaceRecord| known.id == face.id)
            {
                faces.push(face);
            }
        }
    }
    Ok((edge, faces))
}

pub(crate) fn face_plane_normal(
    face: &FaceRecord,
    near: Vec3,
) -> Result<(Vec3, Vec3), KernelRefusal> {
    let projection = crate::project_point_to_surface(&face.surface, near)
        .or_refuse(KernelStage::Classify, "face_projection")?;
    let normal = face
        .surface
        .normal(projection.u, projection.v)
        .or_refuse(KernelStage::Classify, "face_normal")?;
    let outward = if face.same_sense {
        normal
    } else {
        normal.scale(-1.0)
    };
    Ok((projection.point, outward))
}

/// The in-plane direction from the edge into the face's material at
/// `construct_point`.  The SIGN is resolved by trim-inside probing at
/// `probe_point` (for closed circular edges the edge start IS the seam
/// vertex, where the wall face's UV probe lands on the trim boundary —
/// the circle's midpoint is far from the seam, and rotational symmetry
/// transports the sign back to the start meridian).
pub(crate) fn into_face_direction(
    face: &FaceRecord,
    construct_point: Vec3,
    construct_tangent: Vec3,
    probe_point: Vec3,
    probe_tangent: Vec3,
    probe_step: f64,
) -> Result<Vec3, KernelRefusal> {
    let (_, probe_outward) = face_plane_normal(face, probe_point)?;
    let probe_candidate = probe_outward
        .cross(probe_tangent)
        .normalized()
        .or_refuse(KernelStage::Classify, "probe_candidate")?;
    let (_, construct_outward) = face_plane_normal(face, construct_point)?;
    let construct_candidate = construct_outward
        .cross(construct_tangent)
        .normalized()
        .or_refuse(KernelStage::Classify, "construct_candidate")?;
    for sign in [1.0, -1.0] {
        let probe = probe_point.add(probe_candidate.scale(sign * probe_step));
        let projection = crate::project_point_to_surface(&face.surface, probe)
            .or_refuse(KernelStage::Classify, "probe_projection")?;
        // The in-plane probe leaves a CURVED carrier by the sagitta
        // (~step²·curvature); the trim classification below is what
        // actually decides the side, so the distance gate only rejects
        // far-away wrap-around projections.
        if projection.distance <= probe_step * 0.5 {
            let class = crate::parameter_point_in_face(
                face,
                crate::Vec2 {
                    x: projection.u,
                    y: projection.v,
                },
                1e-6,
            )
            .or_refuse(KernelStage::Classify, "probe_trim_class")?;
            if class == crate::PolygonClass::Inside {
                return Ok(construct_candidate.scale(sign));
            }
        }
    }
    Err(support_direction_refusal(face))
}

/// Neither in-plane probe landed inside the face's trim, so the side the
/// material is on could not be read: the geometry at this station does not
/// decide it, and the direction is refused rather than guessed.
fn support_direction_refusal(face: &FaceRecord) -> KernelRefusal {
    KernelRefusal::ill_posed(
        KernelStage::Classify,
        "support_direction",
        format!(
            "fillet: could not orient the support direction into face {}",
            face.id
        ),
    )
}

/// Probe stations, as edge fractions and in the order tried, for fixing a
/// face's into-material sign.  The midpoint leads — it is the one station a
/// closed edge's seam vertex is guaranteed not to sit on — and the rest walk
/// outward from it.
const SIGN_PROBE_FRACTIONS: [f64; 9] = [
    0.5, 0.25, 0.75, 0.375, 0.625, 0.125, 0.875, 0.0625, 0.9375,
];

/// [`into_face_direction`] with the probe station chosen rather than fixed:
/// `station(fraction)` yields the edge point and tangent at that fraction of
/// the edge, and the first station whose trim probe answers wins.
///
/// One station is not always enough.  Where the face PINCHES to zero width
/// against this edge — an inner loop TANGENT to the face's own outer boundary,
/// which is what a bore mouth touching a neighbouring fillet's contact line
/// makes — both in-plane probe signs land outside the trim and the orientation
/// reads as unknowable.  It is not: every other station on the same edge
/// answers, and the sign carries back along the edge, because the material
/// never swaps sides of a face mid-edge (`scan_dihedral` rests on the same
/// continuity, and `analyze_edge`'s paths transport it exactly — rotation about
/// the axis for a circular edge, translation for a straight one).  Probing more
/// stations is what keeps a tangency a geometric coincidence instead of a
/// refusal.
fn into_face_direction_probed(
    face: &FaceRecord,
    construct_point: Vec3,
    construct_tangent: Vec3,
    station: &dyn Fn(f64) -> Result<(Vec3, Vec3), KernelRefusal>,
    probe_step: f64,
) -> Result<Vec3, KernelRefusal> {
    let mut refusal = None;
    for fraction in SIGN_PROBE_FRACTIONS {
        let Ok((probe_point, probe_tangent)) = station(fraction) else {
            continue;
        };
        match into_face_direction(
            face,
            construct_point,
            construct_tangent,
            probe_point,
            probe_tangent,
            probe_step,
        ) {
            Ok(direction) => return Ok(direction),
            Err(error) => refusal = Some(error),
        }
    }
    Err(refusal.unwrap_or_else(|| support_direction_refusal(face)))
}

pub(super) enum EdgePath {
    Straight {
        direction: Vec3,
        length: f64,
    },
    /// Circular edge: the blend is a revolve of the cross-section about
    /// the circle's axis.  The exact wedge requires both mating faces to be
    /// rotationally symmetric about that axis AND to have straight meridians
    /// (plane/cylinder/cone/degree-1 revolution).
    Circular {
        center: Vec3,
        axis: Vec3,
        sweep: f64,
    },
}

pub(super) struct EdgeCross {
    pub(super) start: Vec3,
    pub(super) path: EdgePath,
    pub(super) into_first: Vec3,
    pub(super) into_second: Vec3,
    pub(super) convex: bool,
}

pub(super) fn analyze_edge(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
) -> Result<EdgeCross, KernelRefusal> {
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "radius",
            "fillet: radius must be positive",
        ));
    }
    let (edge, faces) = edge_and_faces(solid, edge_id)?;
    if faces.len() != 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "edge_faces",
            format!(
                "fillet: edge {edge_id} borders {} faces (expected 2)",
                faces.len()
            ),
        ));
    }
    if faces.iter().all(|face| {
        matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Sphere { .. })
        )
    }) {
        // The local tangent-line wedge used by the exact tool has conical
        // side walls, so it is not cosurface with a spherical meridian.  Send
        // this pair through the shared rolling-ball strip + direct trim path.
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "spherical_meridians",
            "fillet: spherical meridians require the general blend surgery",
        ));
    }
    let evaluate = |t: f64| {
        edge.curve
            .evaluate(t)
            .or_refuse(KernelStage::Classify, "edge_evaluate")
    };
    let start = evaluate(edge.t0)?;
    let end = evaluate(edge.t1)?;
    let middle = evaluate((edge.t0 + edge.t1) * 0.5)?;
    let chord = end.sub(start);
    // Straightness is a GEOMETRIC property: every point lies on the chord
    // line, regardless of parameterization. Checking only the parameter-
    // midpoint against the geometric midpoint fails on boolean-reparameterized
    // straight edges (non-uniform knots shift the param-midpoint) — a curved
    // edge still fails because its samples leave the chord line.
    let straight = chord.length() > 1e-9 && {
        let dir = chord.scale(1.0 / chord.length());
        let tol = 1e-6 * (1.0 + chord.length());
        (0..=8).all(|i| {
            let t = edge.t0 + (edge.t1 - edge.t0) * i as f64 / 8.0;
            edge.curve
                .evaluate(t)
                .map(|p| {
                    let d = p.sub(start);
                    d.sub(dir.scale(d.dot(dir))).length() <= tol
                })
                .unwrap_or(false)
        })
    };
    let path = if straight {
        let length = chord.length();
        EdgePath::Straight {
            direction: chord.scale(1.0 / length),
            length,
        }
    } else {
        // Circular edge (full circle when start == end): fit the circle
        // through three samples and verify the rest lie on it.
        let quarter = evaluate(edge.t0 + (edge.t1 - edge.t0) * 0.25)?;
        let center = crate::analytic_surface::circumcenter(start, quarter, middle)
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "edge_path_fit",
                    "fillet: edge is neither straight nor circular",
                )
            })?;
        let radius = start.sub(center).length();
        let axis = quarter
            .sub(center)
            .cross(middle.sub(center))
            .normalized()
            .map_err(|_| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "circular_axis",
                    "fillet: circular edge axis is degenerate",
                )
            })?;
        for index in 0..=16 {
            let point = evaluate(edge.t0 + (edge.t1 - edge.t0) * index as f64 / 16.0)?;
            let radial = point.sub(center);
            if (radial.length() - radius).abs() > 1e-6 * (1.0 + radius)
                || radial.dot(axis).abs() > 1e-6 * (1.0 + radius)
            {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "edge_path_samples",
                    "fillet: edge is neither straight nor circular",
                ));
            }
        }
        let closed = start.sub(end).length() <= 1e-6 * (1.0 + radius);
        let sweep = if closed {
            std::f64::consts::TAU
        } else {
            let from = start.sub(center);
            let to = end.sub(center);
            let mut angle = from.cross(to).dot(axis).atan2(from.dot(to));
            if angle <= 0.0 {
                angle += std::f64::consts::TAU;
            }
            angle
        };
        EdgePath::Circular {
            center,
            axis,
            sweep,
        }
    };
    for face in &faces {
        let analytic = face.surface.analytic();
        let supported = match (&path, analytic) {
            (EdgePath::Straight { .. }, Some(crate::AnalyticSurface::Plane { .. })) => true,
            (EdgePath::Circular { axis, .. }, Some(surface)) => {
                // Rotational symmetry about the edge axis transports the
                // cross-section around the rim; a straight carrier meridian
                // is additionally required below.
                match surface {
                    crate::AnalyticSurface::Plane { u_dir, v_dir, .. } => {
                        u_dir
                            .cross(*v_dir)
                            .normalized()
                            .map(|normal| normal.cross(*axis).length() <= 1e-6)
                            == Ok(true)
                    }
                    // Revolving the planar wedge cross-section is exact only
                    // when each mate has a STRAIGHT meridian.  A sphere or
                    // torus can share the edge's rotation axis, but its
                    // meridian is curved: `cross_section_profile` would use
                    // the tangent LINE as though it lay on that carrier and
                    // produce a visibly non-tangent blend.  Those carriers
                    // belong in the general rolling-ball march.
                    crate::AnalyticSurface::RuledRevolution { frame, .. } => {
                        frame.axis.cross(*axis).length() <= 1e-6
                    }
                    crate::AnalyticSurface::Revolution {
                        frame, generatrix, ..
                    } if generatrix.degree == 1 => frame.axis.cross(*axis).length() <= 1e-6,
                    _ => false,
                }
            }
            _ => false,
        };
        if !supported {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "exact_tool_carriers",
                "fillet: edge/face combination not supported by the exact tool (straight edges \
                 need planar faces; circular edges need straight-meridian faces rotationally \
                 symmetric about the edge axis)",
            ));
        }
    }
    let (_, first_outward) = face_plane_normal(faces[0], start)?;
    let (_, second_outward) = face_plane_normal(faces[1], start)?;
    if first_outward.cross(second_outward).length() <= 1e-6 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "tangent_faces",
            "fillet: faces are tangent along the edge",
        ));
    }
    let tangent_at = |point: Vec3| -> Result<Vec3, KernelRefusal> {
        match &path {
            EdgePath::Straight { direction, .. } => Ok(*direction),
            EdgePath::Circular { center, axis, .. } => axis
                .cross(point.sub(*center))
                .normalized()
                .or_refuse(KernelStage::Classify, "edge_tangent"),
        }
    };
    let scale = match &path {
        EdgePath::Straight { length, .. } => *length,
        EdgePath::Circular { center, .. } => start.sub(*center).length(),
    };
    let probe_step = (scale * 0.05).min(radius * 0.5).max(1e-6);
    let start_tangent = tangent_at(start)?;
    let station = |fraction: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
        let point = evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)?;
        Ok((point, tangent_at(point)?))
    };
    let into_first =
        into_face_direction_probed(faces[0], start, start_tangent, &station, probe_step)?;
    let into_second =
        into_face_direction_probed(faces[1], start, start_tangent, &station, probe_step)?;
    // Convex edge: the material corner points along the inward bisector,
    // so a probe nudged along the OUTWARD bisector leaves the solid.
    // Convex edge: one face's into-material direction dives BELOW the
    // other face's plane (dot with its outward normal negative).  At a
    // concave seam (boss cove) the into-directions run along the other
    // face's outward side instead.  (An air-probe on the outward bisector
    // cannot tell these apart — it lands in air for both.)
    let convex = into_first.dot(second_outward) < -1e-9 || into_second.dot(first_outward) < -1e-9;
    Ok(EdgeCross {
        start,
        path,
        into_first,
        into_second,
        convex,
    })
}

/// How an edge's dihedral behaves along its whole length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DihedralProfile {
    /// Smallest interior (material) angle over the scan, radians.
    pub(crate) min_angle: f64,
    /// Largest interior (material) angle over the scan, radians.
    pub(crate) max_angle: f64,
    /// A sample read strictly convex (interior angle < π − angular).
    pub(crate) any_convex: bool,
    /// A sample read strictly concave (interior angle > π + angular).
    pub(crate) any_concave: bool,
    /// Samples whose surface data could be evaluated.
    pub(crate) samples: usize,
    /// The edge fraction of the first sample whose sense differs from the
    /// first classified sample — where the flip happens, for the message.
    pub(crate) flip_fraction: Option<f64>,
}

impl DihedralProfile {
    /// **Mixed concavity**: convex along part of the edge and concave along the
    /// rest.  No single local blend rule serves such an edge — a rolling ball
    /// would have to move from inside the material to outside it partway along
    /// — which is why OCCT classifies it `ChFiDS_Mixed` and aborts the whole
    /// offset by name (`BRepOffset_Analyse.cxx:149-272`,
    /// `BRepOffset_MakeOffset.cxx:896-903`).
    pub(crate) fn is_mixed(&self) -> bool {
        self.any_convex && self.any_concave
    }
}

/// Scan an edge's dihedral along its length — our answer to OCCT's 23-sample
/// `CheckMixedContinuity`.
///
/// At each station the two faces' OUTWARD normals `n1, n2` and the edge tangent
/// `T` give each face's into-material direction as `d_i = ±(n_i × T)`; the sign
/// is fixed ONCE, at the edge midpoint, by the same trim probe the exact tool
/// already trusts (`into_face_direction`), and transported along the edge by
/// continuity — the material never swaps sides of a face mid-edge.  The
/// interior (material) angle then follows from `d1` and `d2`:
///
/// * `d1 · n2 < 0` — face 1's material dives BELOW face 2, so the wedge is the
///   one the two into-directions span: **convex**, interior angle
///   `angle(d1, d2) < π`.
/// * `d1 · n2 > 0` — the material runs along face 2's outward side:
///   **concave**, interior angle `2π − angle(d1, d2) > π`.
///
/// That is exactly the rule `analyze_edge` already applies at ONE point,
/// evaluated at every station instead — which is the whole difference between
/// having this class and not having it.
///
/// Unlike OCCT we do not copy their 23 samples or their 0.1 rad: the count is
/// this module's own march seed density and the angular band comes from
/// [`crate::KernelTolerances`] for the solid, so the test scales with the part
/// like every other tolerance here.
pub(crate) fn scan_dihedral(
    solid: &BrepSolid,
    edge_id: u64,
) -> Result<DihedralProfile, KernelRefusal> {
    let (edge, faces) = edge_and_faces(solid, edge_id)?;
    if faces.len() != 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "scan_edge_faces",
            format!(
                "dihedral scan: edge {edge_id} borders {} faces (expected 2)",
                faces.len()
            ),
        ));
    }
    let angular = crate::KernelTolerances::for_solid(solid, 1e-7).angular;
    let span = edge.t1 - edge.t0;
    // The edge's unit tangent, read as the one-sided limit where its
    // parameterization is stationary (`NurbsCurve::unit_tangent`); the extended
    // sibling keeps the extended read this scan always made.
    let tangent_at = |t: f64| -> Result<Vec3, KernelRefusal> {
        edge.curve
            .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
            .map(|(_, tangent)| tangent)
            .map_err(|error| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "scan_tangent",
                    format!("dihedral scan: edge {edge_id}: {error}"),
                )
            })
    };
    let evaluate = |t: f64| {
        edge.curve
            .evaluate(t)
            .or_refuse(KernelStage::Classify, "scan_evaluate")
    };
    // Fix each face's into-material sign ONCE, at the parameter midpoint, with
    // the trim probe.  `into_face_direction` wants a construct point and a
    // separate probe point; the midpoint serves as both here because a scan has
    // no seam convention to respect.
    let middle_t = edge.t0 + span * 0.5;
    let middle = evaluate(middle_t)?;
    let middle_tangent = tangent_at(middle_t)?;
    let scale = evaluate(edge.t0)?
        .sub(evaluate(edge.t1)?)
        .length()
        .max(1.0);
    let probe_step = (scale * 0.05).max(1e-6);
    let station = |fraction: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
        let t = edge.t0 + span * fraction;
        Ok((evaluate(t)?, tangent_at(t)?))
    };
    let mut signs = [1.0f64; 2];
    for (index, face) in faces.iter().enumerate() {
        let into =
            into_face_direction_probed(face, middle, middle_tangent, &station, probe_step)?;
        let (_, outward) = face_plane_normal(face, middle)?;
        let candidate = outward
            .cross(middle_tangent)
            .normalized()
            .or_refuse(KernelStage::Classify, "scan_candidate")?;
        signs[index] = if candidate.dot(into) >= 0.0 { 1.0 } else { -1.0 };
    }
    const SAMPLES: usize = 32;
    let mut profile = DihedralProfile {
        min_angle: f64::INFINITY,
        max_angle: f64::NEG_INFINITY,
        any_convex: false,
        any_concave: false,
        samples: 0,
        flip_fraction: None,
    };
    let mut first_sense: Option<bool> = None;
    for step in 0..=SAMPLES {
        let fraction = step as f64 / SAMPLES as f64;
        let t = edge.t0 + span * fraction;
        let Ok(point) = edge.curve.evaluate(t) else {
            continue;
        };
        let Ok(tangent) = tangent_at(t) else {
            continue;
        };
        let mut normals = [Vec3::default(); 2];
        let mut into = [Vec3::default(); 2];
        let mut usable = true;
        for (index, face) in faces.iter().enumerate() {
            let Ok((_, outward)) = face_plane_normal(face, point) else {
                usable = false;
                break;
            };
            let Ok(candidate) = outward.cross(tangent).normalized() else {
                usable = false;
                break;
            };
            normals[index] = outward;
            into[index] = candidate.scale(signs[index]);
        }
        if !usable {
            continue;
        }
        profile.samples += 1;
        let opening = into[0].dot(into[1]).clamp(-1.0, 1.0).acos();
        // Either face's verdict alone decides the sense; averaging the two
        // keeps a near-tangent station from flickering on one of them.
        let sense = into[0].dot(normals[1]) + into[1].dot(normals[0]);
        let interior = if sense < 0.0 {
            opening
        } else {
            std::f64::consts::TAU - opening
        };
        profile.min_angle = profile.min_angle.min(interior);
        profile.max_angle = profile.max_angle.max(interior);
        let flat = std::f64::consts::PI;
        let classified = if interior < flat - angular {
            profile.any_convex = true;
            Some(true)
        } else if interior > flat + angular {
            profile.any_concave = true;
            Some(false)
        } else {
            None
        };
        if let Some(current) = classified {
            match first_sense {
                None => first_sense = Some(current),
                Some(known) if known != current && profile.flip_fraction.is_none() => {
                    profile.flip_fraction = Some(fraction);
                }
                _ => {}
            }
        }
    }
    // OCCT requires at least half its samples to have converged before it
    // trusts a `Mixed` verdict (`BRepOffset_Analyse.cxx:263-266`); the same
    // rule keeps a scan that mostly failed to evaluate from naming a class.
    if profile.samples * 2 < SAMPLES {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "scan_coverage",
            format!(
                "dihedral scan: only {} of {} stations on edge {edge_id} could be evaluated",
                profile.samples,
                SAMPLES + 1
            ),
        ));
    }
    Ok(profile)
}

/// Refuse a MIXED-concavity edge by name, the way OCCT does.
///
/// Silent when the scan cannot run (an edge that does not border exactly two
/// faces is already refused elsewhere, with a better message) and when the edge
/// reads one sense along its whole length — which is every edge in the corpus.
pub(super) fn check_mixed_concavity(
    solid: &BrepSolid,
    edge_id: u64,
    entry: &str,
) -> Result<(), KernelRefusal> {
    let Ok(profile) = scan_dihedral(solid, edge_id) else {
        return Ok(());
    };
    if !profile.is_mixed() {
        return Ok(());
    }
    Err(KernelRefusal::ill_posed(
        KernelStage::Classify,
        "mixed_concavity",
        format!(
            "{entry}: edge {edge_id} has MIXED concavity — its interior angle runs from {:.3}° to \
             {:.3}° and crosses flat{}, so it is convex along part of its length and concave along \
             the rest. A rolling ball would have to pass from inside the material to outside it \
             partway along, so no single blend serves this edge: split it where the faces become \
             tangent and blend the parts separately.",
            profile.min_angle.to_degrees(),
            profile.max_angle.to_degrees(),
            profile
                .flip_fraction
                .map(|fraction| format!(" at about {:.0}% along", fraction * 100.0))
                .unwrap_or_default(),
        ),
    ))
}

/// Rodrigues rotation of `v` about the unit `axis` by `angle`.
fn rotate_about(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    v.scale(cos)
        .add(axis.cross(v).scale(sin))
        .add(axis.scale(axis.dot(v) * (1.0 - cos)))
}

/// The faces joined to `face` across a boundary edge they meet TANGENTIALLY —
/// outward normals parallel and same-sense along the whole shared edge, inside
/// the solid's own angular band.  A fillet's own blend face joins each of its
/// mates this way by construction, which is what makes a rim tangent to a
/// blend's contact line a smooth continuation of that face rather than a wall.
///
/// One pass over the shell: this face's boundary edges first, then every other
/// face tested against that set, so a large solid costs one face scan and not
/// one per boundary edge.  A neighbour sharing several edges qualifies on ANY
/// of them being tangent — the caller's reach is a BOUND, and this errs the
/// same way it does, toward not refusing.
fn tangent_neighbours<'a>(solid: &'a BrepSolid, face: &FaceRecord) -> Vec<&'a FaceRecord> {
    let angular = crate::KernelTolerances::for_solid(solid, 1e-7).angular;
    let borders: Vec<&EdgeRecord> = face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
        .collect();
    let mut found: Vec<&FaceRecord> = Vec::new();
    for other in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if other.id == face.id {
            continue;
        }
        let shared: Vec<&&EdgeRecord> = borders
            .iter()
            .filter(|border| {
                other
                    .loops
                    .iter()
                    .flat_map(|lp| &lp.coedges)
                    .any(|coedge| coedge.edge_id == border.id)
            })
            .collect();
        let tangent = shared.iter().any(|border| {
            (0..=2).all(|step| {
                let t = border.t0 + (border.t1 - border.t0) * step as f64 / 2.0;
                let Ok(point) = border.curve.evaluate(t) else {
                    return false;
                };
                match (
                    face_plane_normal(face, point),
                    face_plane_normal(other, point),
                ) {
                    (Ok((_, here)), Ok((_, there))) => {
                        here.dot(there) > 0.0 && here.cross(there).length() <= angular
                    }
                    _ => false,
                }
            })
        });
        if tangent {
            found.push(other);
        }
    }
    found
}


/// Whether a tangent neighbour can carry a rolling ball of radius `size`
/// past the join, judged by its curvature along the shared edge.
///
/// A ball of radius r cannot follow a surface whose radius of curvature is r
/// or less: a concave one it does not fit into, and a convex one the ball the
/// two supports place cuts into rather than rests on.  The blended
/// through-hole is the convex case exactly: past r = t/2 the second mouth
/// blend's contact lands on the first blend, a torus of tube radius r, and
/// the ball the bottom plane and the wall position is 2r − t inside it
/// (`closed_heal_tests::a_blended_through_hole_never_reaches_the_cap_lane`).
/// The reported bore mouth is the other side of the line — an r = 0.5 blend
/// rolling onto an r = 4 corner round — and is what tangent neighbours count
/// for at all.
///
/// Judged by the TIGHTEST principal curvature at three stations on every
/// shared edge.  That is a bound, and it errs toward refusing: a neighbour
/// curving tightly only across the ball's path is declined too, and what
/// stands then is the refusal this check gave before tangent neighbours
/// counted.  A station that cannot be projected or differentiated declines
/// the same way.
pub(crate) fn neighbour_carries_ball(
    solid: &BrepSolid,
    face: &FaceRecord,
    neighbour: &FaceRecord,
    size: f64,
) -> bool {
    let limit = size * (1.0 + 1e-6);
    let mut stations = 0usize;
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let shared = neighbour
            .loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .any(|other| other.edge_id == coedge.edge_id);
        if !shared {
            continue;
        }
        let Some(border) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
            continue;
        };
        for step in 0..=2 {
            let t = border.t0 + (border.t1 - border.t0) * step as f64 / 2.0;
            let Ok(point) = border.curve.evaluate(t) else {
                return false;
            };
            let Ok(projection) = crate::project_point_to_surface(&neighbour.surface, point) else {
                return false;
            };
            let Ok((kappa_min, kappa_max)) = neighbour
                .surface
                .principal_curvatures(projection.u, projection.v)
            else {
                return false;
            };
            let tightest = kappa_min.abs().max(kappa_max.abs());
            if tightest > 0.0 && 1.0 / tightest <= limit {
                return false;
            }
            stations += 1;
        }
    }
    stations > 0
}

/// Upper bound on the boundary's projection along a support direction.
/// Positive-weight rational curves lie in their dehomogenized control hull.
/// Subdivide only hulls that straddle the required reach; a sampled maximum
/// is a lower bound and cannot justify refusing a curved support. Unresolved
/// hulls (including split failures) keep their upper bound and do not refuse.
fn boundary_extent_bound(
    curve: &NurbsCurve,
    station: Vec3,
    direction: Vec3,
    required: f64,
    depth: usize,
) -> f64 {
    let mut upper = f64::NEG_INFINITY;
    for p in &curve.control_points {
        let Ok(point) = p.point() else {
            return f64::INFINITY;
        };
        if p.w <= 0.0 {
            return f64::INFINITY;
        }
        upper = upper.max(point.sub(station).dot(direction));
    }
    if upper < required || depth == 20 {
        return upper;
    }
    let Ok([a, b]) = curve.domain() else {
        return upper;
    };
    let mid = (a + b) * 0.5;
    if [a, mid, b].iter().any(|&t| {
        curve
            .evaluate(t)
            .is_ok_and(|p| p.sub(station).dot(direction) >= required)
    }) {
        return upper;
    }
    let Ok((left, right)) = curve.split(mid) else {
        return upper;
    };
    boundary_extent_bound(&left, station, direction, required, depth + 1).max(
        boundary_extent_bound(&right, station, direction, required, depth + 1),
    )
}

/// **The rolling ball must actually TOUCH the faces it is supposed to be
/// tangent to.**
///
/// A ball of radius r tangent to both supports sits in the wedge spanned by the
/// two into-material directions, on its bisector, at distance `r/sin(φ/2)` from
/// the edge — φ being the angle between `into_first` and `into_second`, which
/// is the wedge the ball occupies for a convex edge (the material) and for a
/// concave one (the void) alike.  Its contact point on each support is the foot
/// of the perpendicular from that centre:
///
/// > **reach = r · cot(φ/2), measured from the edge ALONG that face's
/// > into-material direction. Both contact points must lie ON their support
/// > face.**
///
/// A chamfer's setback is the same length: §6.11 replaces the arc with its
/// chord between the SAME two tangency points.
///
/// When the contact point is off the support, the ball never touches it and
/// what gets built is tangent to nothing.  On a 20-cube at r = 25 both contacts
/// land 5 units off the solid and the exact tool still returns a *validating*
/// five-face solid — a corner CUT by a cylinder whose axis lies outside the
/// part — missing the quarter-round closed form by 0.63 % (pinned by
/// `oversized_radius_refuses_instead_of_cutting_tangent_to_nothing` in this
/// module's `tests.rs`).  The concave twin is worse, and volume cannot see it
/// at all: an L-block's pad matches the UNCLIPPED closed form to 1e-6 at every
/// radius, so past the wall length it simply overhangs the part — bounding box
/// 40 → 45 at r = 25, → 60 at r = 40.
///
/// **The frame is the solid the SELECTION was made on, never a partly blended
/// one.**  Two fillets meeting on a shared face legitimately push each other's
/// contact off the shrunken remainder, and the pinch they produce is exact:
/// two r = 12 rounds on a 20-wide face ridge at z = 8 + √(144−4) = 19.832,
/// which is what the kernel builds.  Checking against the evolving solid would
/// refuse that.
///
/// Silent on anything outside the exact tool's domain (`analyze_edge` errs) —
/// the general lanes carry their own gates — and on a contact point the trim
/// classifier cannot place, so a new refusal never rests on a "don't know".
pub(super) fn check_support_extent(
    solid: &BrepSolid,
    edge_id: u64,
    size: f64,
    entry: &str,
) -> Result<(), KernelRefusal> {
    match support_extent(solid, edge_id, size, entry)? {
        SupportExtent::Fits => Ok(()),
        SupportExtent::ConcaveClip { refusal, .. } => Err(refusal),
    }
}

/// [`check_support_extent`] for a selection of ONE edge, which also admits a
/// concave overrun the exact cutter clips (lane F row 5): the face outline
/// prism of [`concave_clip_face`]. A group keeps the refusal — its network
/// would build the pad unclipped.
pub(super) fn check_support_extent_clipping(
    solid: &BrepSolid,
    edge_id: u64,
    size: f64,
    entry: &str,
) -> Result<(), KernelRefusal> {
    support_extent(solid, edge_id, size, entry).map(|_| ())
}

/// The planar support face whose outline clips a concave edge's fill, when
/// the fill's contact runs past that face's outer boundary into air and
/// nowhere else (lane F row 5). `None` for every other edge, including one
/// that fits.
pub(super) fn concave_clip_face(solid: &BrepSolid, edge_id: u64, size: f64) -> Option<u64> {
    match support_extent(solid, edge_id, size, "") {
        Ok(SupportExtent::ConcaveClip { face, .. }) => Some(face),
        _ => None,
    }
}

enum SupportExtent {
    Fits,
    /// A concave edge overruns only `face`, a plane whose outline neighbours
    /// are all planes parallel to its normal meeting it at sharp convex
    /// edges: the fill clipped by that outline's prism is the blend, and
    /// `refusal` is what the strict check says.
    ConcaveClip { face: u64, refusal: KernelRefusal },
}

fn support_extent(
    solid: &BrepSolid,
    edge_id: u64,
    size: f64,
    entry: &str,
) -> Result<SupportExtent, KernelRefusal> {
    // Diagnostic (lane F, 2026-10-15).
    if std::env::var_os("BREP_NO_SUPPORT_EXTENT").is_some() {
        return Ok(SupportExtent::Fits);
    }
    let Ok(cross) = analyze_edge(solid, edge_id, size) else {
        return Ok(SupportExtent::Fits);
    };
    let Ok((_, faces)) = edge_and_faces(solid, edge_id) else {
        return Ok(SupportExtent::Fits);
    };
    if faces.len() != 2 {
        return Ok(SupportExtent::Fits);
    }
    let phi = cross
        .into_first
        .dot(cross.into_second)
        .clamp(-1.0, 1.0)
        .acos();
    // Tangent supports are already refused by `analyze_edge`; a degenerate
    // wedge here would only make `reach` meaningless.
    if !(phi > 1e-9) || phi >= std::f64::consts::PI - 1e-9 {
        return Ok(SupportExtent::Fits);
    }
    let reach = size / (phi * 0.5).tan();
    if !reach.is_finite() || reach <= 0.0 {
        return Ok(SupportExtent::Fits);
    }
    let (center, axis, sweep) = match &cross.path {
        EdgePath::Circular {
            center,
            axis,
            sweep,
        } => (*center, *axis, *sweep),
        EdgePath::Straight { .. } => (Vec3::default(), Vec3::default(), 0.0),
    };
    let straight = matches!(cross.path, EdgePath::Straight { .. });
    const SAMPLES: usize = 8;
    // Every station is read before deciding (lane F row 2, 2026-10-15;
    // `BREP_SUPPORT_EXTENT_STRICT=1` restores the refusal at the first overrun
    // station, for comparison). A CONVEX edge whose ball runs off ONE support while
    // it still touches the other is admitted at that station when the overrun leaves
    // that support across a SHARP CONVEX edge — out of the part into air, so the
    // overrunning wall removes only material the solid does not have and the third
    // face it meets clips it. An overrun across a concave or tangent edge hands the
    // ball to a third face it would cut into (a bore's last wall ending at the far
    // mouth's blend) and still refuses; so does a station where the ball runs off
    // BOTH supports, tangent to nothing (the 20-cube at r = 25 below), and a concave
    // edge at any unsupported station, whose pad overhangs the part unclipped
    // (lCount_413 +1.0 % volume).
    let partial = std::env::var_os("BREP_SUPPORT_EXTENT_STRICT").is_none();
    let mut overrun = [0u8; SAMPLES + 1];
    let angular = crate::KernelTolerances::for_solid(solid, 1e-7).angular;
    // The boundary edge the overrun leaves its support across: the one
    // nearest the segment from the station to the contact point.
    let exits_into_air = |edges: &[&EdgeRecord], station: Vec3, contact: Vec3| {
        let span = contact.sub(station);
        let to_segment = |point: Vec3| {
            let along = (point.sub(station).dot(span) / span.dot(span)).clamp(0.0, 1.0);
            point.sub(station.add(span.scale(along))).length()
        };
        let exit = edges
            .iter()
            .filter(|border| border.id != edge_id)
            .map(|border| {
                let nearest = (0..=64)
                    .filter_map(|step| {
                        let t = border.t0 + (border.t1 - border.t0) * step as f64 / 64.0;
                        border.curve.evaluate(t).ok().map(to_segment)
                    })
                    .fold(f64::INFINITY, f64::min);
                (border.id, nearest)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));
        exit.is_some_and(|(exit_id, _)| {
            scan_dihedral(solid, exit_id)
                .is_ok_and(|profile| profile.samples > 0 && profile.max_angle < std::f64::consts::PI - angular)
        })
    };
    let mut first_refusal: Option<KernelRefusal> = None;
    // Which support each overrun station ran off (bit 0 = first, 1 = second).
    let mut overrun_faces = [0u8; SAMPLES + 1];
    for (support, (face, into)) in
        [(faces[0], cross.into_first), (faces[1], cross.into_second)].into_iter().enumerate()
    {
        // The face's own boundary, in 3D.  The test below is a BOUND, not a
        // containment test, and deliberately so: `project_point_to_surface`
        // clamps to the carrier's parameter rectangle, so a contact point past
        // a box face's own patch comes back AT the boundary with a residual —
        // exactly the census case — and `parameter_point_in_face` then
        // classifies the clamped point Inside.  Measuring how far the face
        // itself reaches along the into-direction needs no inversion, and it
        // gives a face that reaches far enough somewhere the benefit of the
        // doubt even if it does not reach there at this station. Samples can
        // establish sufficient reach, but only a control-hull upper bound on
        // every trimmed boundary curve can establish insufficient reach.
        //
        // The reach is measured over this face's boundary AND over the
        // boundary of every face joined to it TANGENTIALLY.  A support that
        // runs off a face across a G1 join has not run out of support: the
        // neighbour continues the same smooth wall and the rolling ball rolls
        // straight onto it.  That is not a corner case here — it is the
        // reported one: a bore mouth whose rim is TANGENT to an adjacent
        // fillet's contact line pinches its own face to zero width at the
        // touch point, so the face's own reach there is exactly 0 and every
        // radius would be refused.  One hop is what a single G1 edge buys and
        // all this check needs; a SHARP neighbour is deliberately not
        // included, because running off one of those is the case this check
        // exists to refuse.  Nor is a tangent neighbour the ball cannot
        // actually roll onto — one curving as tightly as the ball or tighter
        // (`neighbour_carries_ball`): the far mouth's own round on a blended
        // through-hole is G1 with the wall and still no wall to lie on.
        let mut boundary: Vec<Vec3> = Vec::new();
        let mut boundary_edges = Vec::new();
        let mut sample_face = |sampled: &FaceRecord, boundary: &mut Vec<Vec3>| {
            for loop_record in &sampled.loops {
                for coedge in &loop_record.coedges {
                    let Some(border) = solid.edges.iter().find(|e| e.id == coedge.edge_id) else {
                        continue;
                    };
                    boundary_edges.push(border);
                    for step in 0..=SAMPLES {
                        let t = border.t0 + (border.t1 - border.t0) * step as f64 / SAMPLES as f64;
                        if let Ok(point) = border.curve.evaluate(t) {
                            boundary.push(point);
                        }
                    }
                }
            }
        };
        sample_face(face, &mut boundary);
        for neighbour in tangent_neighbours(solid, face) {
            if !neighbour_carries_ball(solid, face, neighbour, size) {
                continue;
            }
            sample_face(neighbour, &mut boundary);
        }
        if boundary.is_empty() {
            continue;
        }
        for step in 0..=SAMPLES {
            let fraction = step as f64 / SAMPLES as f64;
            let (station, direction) = match &cross.path {
                EdgePath::Straight { direction, length } if straight => {
                    (cross.start.add(direction.scale(length * fraction)), into)
                }
                _ => {
                    let angle = sweep * fraction;
                    (
                        center.add(rotate_about(cross.start.sub(center), axis, angle)),
                        rotate_about(into, axis, angle),
                    )
                }
            };
            let extent = boundary
                .iter()
                .map(|point| point.sub(station).dot(direction))
                .fold(f64::NEG_INFINITY, f64::max);
            if !extent.is_finite() {
                continue;
            }
            // Equality is the LIMIT case and it is exact: at r = the support's
            // own width the contact lands on the far boundary, the support is
            // consumed whole, and the network builds it to the closed form
            // (5F/9E/6V on a cube, 1e-11).  Refuse strictly beyond the band
            // the surgery SNAPS a rail across (`consumed_band`): a wider
            // slack admits a rail that runs off its face and is neither
            // snapped nor refused.
            if reach <= extent + crate::blend::consumed_band(solid) {
                continue;
            }
            let required = reach - crate::blend::consumed_band(solid);
            let extent = boundary_edges
                .iter()
                .map(|border| match crate::step::edge_subcurve(border) {
                    Ok(curve) => boundary_extent_bound(&curve, station, direction, required, 0),
                    Err(_) => f64::INFINITY,
                })
                .fold(f64::NEG_INFINITY, f64::max);
            if extent >= required {
                continue;
            }
            let refusal = KernelRefusal::input(
                KernelStage::Classify,
                "support_extent",
                format!(
                    "{entry}: {size} is too large for the faces it must lie on — the blend would \
                     meet face {} at {reach:.6} from the edge (reach = size·cot(φ/2), φ = {:.3}°), \
                     but that face only reaches {extent:.6} there. A rolling ball that cannot touch \
                     both supports is tangent to neither, so what would be built is not a blend of \
                     this edge: reduce the size, or select the neighbouring faces' edges too so the \
                     blend has somewhere to run.",
                    face.id,
                    phi.to_degrees(),
                ),
            );
            if !partial {
                return Err(refusal);
            }
            overrun_faces[step] |= 1 << support;
            let contact = station.add(direction.scale(reach));
            overrun[step] += if exits_into_air(&boundary_edges, station, contact) { 1 } else { 2 };
            first_refusal.get_or_insert(refusal);
        }
    }
    let Some(refusal) = first_refusal else {
        return Ok(SupportExtent::Fits);
    };
    if cross.convex {
        return if overrun.contains(&2) { Err(refusal) } else { Ok(SupportExtent::Fits) };
    }
    // A concave edge: its pad is admitted only where one clip makes it exact.
    let ran_off = overrun_faces.iter().fold(0u8, |all, bits| all | bits);
    let support = match ran_off {
        0b01 => 0,
        0b10 => 1,
        _ => return Err(refusal),
    };
    match concave_clip_outline(solid, faces[support], edge_id, angular) {
        Some(_) => Ok(SupportExtent::ConcaveClip { face: faces[support].id, refusal }),
        None => Err(refusal),
    }
}

/// The outline of `face` that clips a concave edge's fill: every loop that
/// does not hold `edge_id`, when `face` is a plane and every edge of those
/// loops is a sharp convex edge to a wall parallel to `face`'s normal (a plane
/// containing it, or a cylinder about an axis along it), so the region's prism
/// along that normal IS the walls the fill runs into: the outer loop's side
/// planes and each hole's own wall (lane F). The loops' indices in
/// `face.loops`, the outer loop (the largest projected area) first, then the
/// holes.
pub(super) fn concave_clip_outline(
    solid: &BrepSolid,
    face: &FaceRecord,
    edge_id: u64,
    angular: f64,
) -> Option<Vec<usize>> {
    if !matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
        return None;
    }
    let mut loops: Vec<usize> = (0..face.loops.len())
        .filter(|&index| face.loops[index].coedges.iter().all(|coedge| coedge.edge_id != edge_id))
        .collect();
    if loops.is_empty() || loops.len() + 1 != face.loops.len() {
        return None;
    }
    let anchor = solid.edges.iter().find(|edge| edge.id == edge_id)?.curve.evaluate(0.0).ok()?;
    let (_, normal) = face_plane_normal(face, anchor).ok()?;
    let mut areas = Vec::with_capacity(loops.len());
    for &index in &loops {
        let mut samples = Vec::new();
        for coedge in &face.loops[index].coedges {
            let (border, mates) = edge_and_faces(solid, coedge.edge_id).ok()?;
            let side = mates.iter().find(|mate| mate.id != face.id)?;
            let wall = match side.surface.analytic() {
                Some(crate::AnalyticSurface::Plane { .. }) => {
                    let (_, side_normal) = face_plane_normal(side, border.curve.evaluate(border.t0).ok()?).ok()?;
                    side_normal.dot(normal).abs() <= angular
                }
                Some(crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }) => {
                    let scale = rho0.abs().max(rho1.abs()).max(1.0);
                    (rho0 - rho1).abs() <= 1e-9 * scale && frame.axis.cross(normal).length() <= angular
                }
                _ => false,
            };
            if !wall {
                return None;
            }
            let sharp = scan_dihedral(solid, border.id)
                .is_ok_and(|profile| profile.samples > 0 && profile.max_angle < std::f64::consts::PI - angular);
            if !sharp {
                return None;
            }
            let curve = crate::step::edge_subcurve(border).ok()?;
            let [t0, t1] = curve.domain().ok()?;
            for step in 0..16 {
                let fraction = if coedge.forward { step as f64 / 16.0 } else { 1.0 - step as f64 / 16.0 };
                samples.push(curve.evaluate(t0 + (t1 - t0) * fraction).ok()?);
            }
        }
        areas.push(crate::polygon::newell_normal(&samples).dot(normal).abs());
    }
    let outer = (0..loops.len()).max_by(|&a, &b| areas[a].total_cmp(&areas[b]))?;
    loops.swap(0, outer);
    Some(loops)
}

/// Lane R1 row 5: the interference check's revolution certificate. On by
/// default (promoted 2026-10-18 on the 98 + 144 census);
/// `BREP_INTERFERENCE_REVOLUTION_CERT=0` (or `off`) samples every body.
pub(crate) fn interference_revolution_cert_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| !matches!(std::env::var("BREP_INTERFERENCE_REVOLUTION_CERT").as_deref(), Ok("0") | Ok("off")))
}


/// A grown face that is the FULL-TURN revolution of its own u = 0 row about
/// an axis: `(axis origin, unit axis, [ρmin, ρmax, zmin, zmax], a point of
/// the box's interior)`, where the box is the meridian (ρ, z) extent of the
/// generatrix's control hull — it holds the generatrix (positive weights) and
/// so, revolved, the whole carrier and every trimmed piece of it. Read from
/// the net alone: the axis from the first column's u circle, the net rebuilt
/// by `make_revolution` from that axis and row and compared to `1e-12` of its
/// size; the row in one meridian half-plane (ρ is then linear on its hull).
/// `None` for anything else, a partial turn included.
fn revolution_band(surface: &crate::NurbsSurface) -> Option<(Vec3, Vec3, [f64; 4], Vec3)> {
    const DELTA: f64 = 1e-12;
    let net = &surface.control_points;
    if surface.degree_u != 2 || net.len() != 9 || net.iter().any(|row| row.len() != net[0].len()) || net[0].len() < 2 {
        return None;
    }
    let at = |i: usize| net[i][0].point().ok();
    let (first, quarter, opposite) = (at(0)?, at(2)?, at(4)?);
    let origin = first.add(opposite).scale(0.5);
    let axis = first.sub(origin).cross(quarter.sub(origin)).normalized().ok()?;
    let generatrix = crate::NurbsCurve::new(surface.degree_v, surface.knots_v.clone(), net[0].clone()).ok()?;
    let rebuilt = crate::make_revolution(origin, axis, &generatrix, std::f64::consts::TAU).ok()?;
    let same_bits = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    if !same_bits(&surface.knots_u, &rebuilt.knots_u) || !same_bits(&surface.knots_v, &rebuilt.knots_v) || rebuilt.degree_u != 2 {
        return None;
    }
    let size = net.iter().flatten().filter_map(|c| c.point().ok()).fold(0f64, |m, q| m.max(q.x.abs()).max(q.y.abs()).max(q.z.abs()));
    if !(size > 0.) || rebuilt.control_points.len() != net.len() || rebuilt.control_points.iter().zip(net).any(|(a, b)| a.len() != b.len()) {
        return None;
    }
    let close = net.iter().flatten().zip(rebuilt.control_points.iter().flatten()).all(|(a, b)| {
        a.w > 0.
            && b.w > 0.
            && (a.w - b.w).abs() <= DELTA * a.w.abs().max(b.w.abs())
            && (a.x / a.w - b.x / b.w).abs() <= DELTA * size
            && (a.y / a.w - b.y / b.w).abs() <= DELTA * size
            && (a.z / a.w - b.z / b.w).abs() <= DELTA * size
    });
    if !close {
        return None;
    }
    // The meridian half-plane of the u = 0 row: through the axis and the
    // first column's radial direction.
    let radial_of = |q: Vec3| {
        let d = q.sub(origin);
        d.sub(axis.scale(d.dot(axis)))
    };
    let x_axis = radial_of(first).normalized().ok()?;
    let normal = axis.cross(x_axis);
    let mut bounds = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
    for control in &net[0] {
        let q = control.point().ok()?;
        let d = q.sub(origin);
        if d.dot(normal).abs() > DELTA * size {
            return None;
        }
        let (x, z) = (d.dot(x_axis), d.dot(axis));
        if !(x >= 0.) {
            return None;
        }
        bounds = [bounds[0].min(x), bounds[1].max(x), bounds[2].min(z), bounds[3].max(z)];
    }
    if !(bounds[1] > bounds[0] || bounds[3] > bounds[2]) {
        return None;
    }
    let interior = origin
        .add(x_axis.scale(0.5 * (bounds[0] + bounds[1])))
        .add(axis.scale(0.5 * (bounds[2] + bounds[3])));
    Some((origin, axis, bounds, interior))
}

/// Does no face of `original` reach into the open box `ρ ∈ (ρmin, ρmax),
/// z ∈ (zmin, zmax)` about `(origin, axis)`, past `tol`? Each face is cleared
/// EXACTLY, never by a sample: a plane perpendicular to the axis by its level,
/// a coaxial cylinder by its radius (the whole carriers), any other face by
/// the meridian extent of its control hull — z is linear on the hull, and ρ
/// convex, so the hull's largest ρ is a control point's.
fn revolution_box_clear(original: &BrepSolid, origin: Vec3, axis: Vec3, [rho0, rho1, z0, z1]: [f64; 4], tol: f64) -> bool {
    const PARALLEL: f64 = 1e-12;
    original.shells.iter().flat_map(|shell| &shell.faces).all(|face| {
        let surface = &face.surface;
        match surface.analytic() {
            Some(crate::AnalyticSurface::Plane { origin: at, u_dir, v_dir, .. }) => {
                if let Ok(normal) = u_dir.cross(*v_dir).normalized() {
                    if normal.cross(axis).length() <= PARALLEL {
                        let level = at.sub(origin).dot(axis);
                        if level >= z1 - tol || level <= z0 + tol {
                            return true;
                        }
                    }
                }
            }
            Some(crate::AnalyticSurface::RuledRevolution { frame, rho0: a, rho1: b, .. }) if a == b => {
                let offset = frame.origin.sub(origin);
                let off_axis = offset.sub(axis.scale(offset.dot(axis))).length();
                if frame.axis.cross(axis).length() <= PARALLEL && off_axis <= tol && (*a >= rho1 - tol || *a <= rho0 + tol) {
                    return true;
                }
            }
            _ => {}
        }
        let (mut low, mut high, mut far) = (f64::INFINITY, f64::NEG_INFINITY, 0f64);
        for control in surface.control_points.iter().flatten() {
            let Ok(q) = control.point() else { return false };
            if !(control.w > 0.) {
                return false;
            }
            let d = q.sub(origin);
            let z = d.dot(axis);
            low = low.min(z);
            high = high.max(z);
            far = far.max(d.sub(axis.scale(z)).length());
        }
        high <= z0 + tol || low >= z1 - tol || far <= rho0 + tol
    })
}

/// The interference verdict for a body whose every grown face is a full-turn
/// revolution band (`revolution_band`) about an axis no face of the original
/// enters inside the band's meridian box `A`: `int A` is connected and misses
/// the original's boundary, so every point of `A` that is not ON the boundary
/// shares the class of one interior point; the bands lie in their `A`, and
/// every fresh edge bounds a band. One classification per band then decides
/// what the edge and face samples would: `Some(true)` clear (no sample can be
/// on the forbidden side), `None` when any condition does not hold (the
/// samples run, as before). Never `Some(false)`: a forbidden interior point
/// is the samples' to report.
fn revolution_interference_certificate(
    original: &BrepSolid,
    result: &BrepSolid,
    grown: &[&FaceRecord],
    fresh_edges: &[u64],
    forbidden: PointClass,
    classifier: &SolidClassifier,
    tol: f64,
) -> Option<bool> {
    if grown.is_empty() {
        return None;
    }
    let mut banded: Vec<u64> = Vec::with_capacity(grown.len());
    for face in grown {
        let (origin, axis, bounds, interior) = revolution_band(&face.surface)?;
        if !revolution_box_clear(original, origin, axis, bounds, tol) {
            return None;
        }
        let class = classifier.classify(interior).ok()?.class;
        if class == forbidden || class == PointClass::On {
            return None;
        }
        banded.push(face.id);
    }
    // Every fresh edge bounds a band: its samples are points of the band.
    let on_band = |edge_id: u64| {
        result.shells.iter().flat_map(|shell| &shell.faces).filter(|face| banded.contains(&face.id)).any(|face| {
            face.loops.iter().flat_map(|l| &l.coedges).any(|coedge| coedge.edge_id == edge_id)
        })
    };
    fresh_edges.iter().all(|&edge_id| on_band(edge_id)).then_some(true)
}

/// **A blend is LOCAL: it may only remove material that is there, or add
/// material where there is none.**
///
/// The march + §6.9 surgery lanes construct the blend surface from the rolling
/// ball and re-trim *the edge's two mating faces* to the contact rails.  That
/// is the whole of their topology work, and it is complete only while the
/// swept volume — the sliver a convex blend removes, the pad a concave one adds
/// — touches nothing but those two mates.  Let a THIRD face cross that volume
/// (a through hole under the rail, a pocket, a neighbouring boss) and the
/// surgery has nothing to say about it: the blend wall is emitted whole and
/// runs straight through the other face, the mate keeps an inner loop that no
/// longer lies inside its own trimmed region, and the result is self-intersecting.
/// `validate` does not see it — every edge still has two uses and every pcurve
/// still lies on its carrier — so without this gate the wrong solid is returned.
///
/// The test needs no intersection: a convex blend's surface lies INSIDE the
/// material it was built on (it is tangent to the boundary along the two rails
/// and strictly interior between them), and a concave blend's surface lies
/// OUTSIDE it.  So classify samples of everything the surgery CREATED against
/// the ORIGINAL solid: a convex blend landing `Out` is geometry in a void, a
/// concave one landing `In` is a pad buried in material.  `On` never violates —
/// a blend that merely touches another face is trimmed correctly by the rails
/// alone.
///
/// Two passes, because they have different sensitivities:
///
/// * **The new EDGES**, sampled densely.  A contact rail is the sharpest probe
///   there is: it is supposed to lie ON its mate, so the moment it runs over a
///   hole in that mate it reads `Out` over a whole span.  This is what catches
///   a SHALLOW interference — a blend whose wall only clips the crown of a hole
///   sweeps a uv patch too thin for any affordable surface grid, but its rail
///   still crosses the opening over a finite length.
/// * **The new FACES**, on an interior uv grid.  This is what catches an
///   interference the rails miss entirely — a void wholly under the blend's
///   arc, touching neither mate.
///
/// Refusing here routes the selection to the boolean cutter
/// (`fillet_or_chamfer_exact`), which imprints and classifies and therefore
/// *does* split the crossing faces and drop the fragments in the void.
///
/// Neither pass classifies at `model`: rails and blend surfaces are FITS
/// through marched stations, so a point of one sits a fit-accuracy away from
/// the carrier it belongs to and the tight `model` band would read that drift
/// as a side.  Each pass takes the kernel's own named contract for its own
/// object — [`KernelTolerances::pcurve_consistency`] for an edge against the
/// carrier it is supposed to lie ON, the tighter
/// [`KernelTolerances::intersection_fit`] for a surface sample, which is not a
/// boundary point at all.  A real violation is a blend radius away from the
/// skin, orders above either band.
///
/// Silent — never a refusal resting on a "don't know" — when the original does
/// not validate (ray classification against a torn boundary is meaningless),
/// when any selected edge's dihedral cannot be scanned or reads mixed, when the
/// selection is neither uniformly convex nor uniformly concave, and when the
/// surgery created nothing new.  A single stray sample never refuses either:
/// two samples must land on the forbidden side.
pub(super) fn check_blend_interference(
    original: &BrepSolid,
    result: &BrepSolid,
    edge_ids: &[u64],
    entry: &str,
) -> Result<(), KernelRefusal> {
    let _timing = crate::blend::BlendTimer::start("timing.interference_ms");
    // The classifier's own counters (builds, band projections, rays) for the
    // scan, printed under `BREP_PROFILE` as `classify.profile blend.interference.*`.
    crate::classification::classify_profile_begin();

    if edge_ids.is_empty() {
        return Ok(());
    }
    let validate_timing = crate::blend::BlendTimer::start("timing.interference.validate_ms");
    let original_invalid = !original.validate_memoised().is_empty();
    validate_timing.stop();
    if original_invalid {
        return Ok(());
    }
    // Which side of the original boundary the new faces may NOT be on.
    let (mut any_convex, mut any_concave) = (false, false);
    for &edge_id in edge_ids {
        let Ok(profile) = scan_dihedral(original, edge_id) else {
            return Ok(());
        };
        if profile.is_mixed() {
            return Ok(());
        }
        any_convex |= profile.any_convex;
        any_concave |= profile.any_concave;
    }
    let forbidden = match (any_convex, any_concave) {
        (true, false) => PointClass::Out,
        (false, true) => PointClass::In,
        _ => return Ok(()),
    };

    let existing: Vec<u64> = original
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.id)
        .collect();
    let grown: Vec<&FaceRecord> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| !existing.contains(&face.id))
        .collect();
    if grown.is_empty() && result.edges.len() <= original.edges.len() {
        return Ok(());
    }

    let policy = crate::KernelTolerances::for_solid(original, 1e-7);
    let report = |label: String, violations: &[Vec3], sampled: usize, what: &str| -> KernelRefusal {
        let side = if forbidden == PointClass::Out {
            "outside the material it was cut from"
        } else {
            "inside material that was already there"
        };
        let sample = violations[0];
        // The trim that would answer this — splitting the crossing faces
        // and dropping the fragments in the void — is a construction the
        // surgery does not perform: a named deferral, read at `Validate`.
        KernelRefusal::unsupported(
            KernelStage::Validate,
            "blend_interference",
            format!(
                "{entry}: the blend is not trimmed where it meets the rest of the solid — {} of {} \
                 samples {what} {label} lie {side} (e.g. ({:.6}, {:.6}, {:.6})). The rolling-ball \
                 surgery re-trims only the blend's two MATING faces, so a third face crossing the \
                 swept volume leaves the blend running through it and the result self-intersecting. \
                 Those faces have to be split along their intersection and the fragments in the \
                 void discarded.",
                violations.len(),
                sampled,
                sample.x,
                sample.y,
                sample.z,
            ),
        )
    };

    // Pass 1 — the new EDGES, densely.  Endpoints are skipped: they are the
    // blend's own vertices, which sit ON the original boundary by construction.
    //
    // Edges get their OWN classifier, at the `pcurve_consistency` band rather
    // than `intersection_fit`.  A contact rail is SUPPOSED to lie on its mate,
    // so every rail sample is a boundary point and the only question is which
    // way its fit drifted — and `pcurve_consistency` is precisely the kernel's
    // named contract for how far a fitted edge may sit from the carrier it
    // belongs to.  Below that band a sample reads `On` and cannot refuse
    // anything; a rail that has actually run out over a hole is a whole blend
    // radius away from the skin, orders above it.
    const EDGE_SAMPLES: usize = 24;
    let edges_timing = crate::blend::BlendTimer::start("timing.interference.edges_ms");
    let inherited: Vec<u64> = original.edges.iter().map(|edge| edge.id).collect();
    // A degenerate edge (a runout's pole, t0 == t1) is a vertex: its 23
    // samples are one point ON the body's boundary, and every one went to a
    // ray (2026-10-17 face ring: 3 poles, 69 rays).
    let fresh: Vec<_> = result
        .edges
        .iter()
        .filter(|edge| !inherited.contains(&edge.id) && !edge.degenerate && edge.t1 != edge.t0)
        .collect();
    // Lane R1 row 5: bands that are full-turn revolutions in a box no face of
    // the original enters are decided by one classification each
    // (`revolution_interference_certificate`); anything else samples.
    if interference_revolution_cert_on() {
        let cert_timing = crate::blend::BlendTimer::start("timing.interference.revolution_cert_ms");
        let band = policy.pcurve_consistency.min(policy.intersection_fit) / 10.0;
        let fresh_ids: Vec<u64> = fresh.iter().map(|edge| edge.id).collect();
        let certified = SolidClassifier::new(original, policy.pcurve_consistency / 10.0)
            .ok()
            .and_then(|classifier| revolution_interference_certificate(original, result, &grown, &fresh_ids, forbidden, &classifier, band));
        cert_timing.stop();
        if certified == Some(true) {
            crate::blend::record_count("count.interference.revolution_certified", 1);
            edges_timing.stop();
            crate::classification::classify_profile_report("blend.interference.edges");
            return Ok(());
        }
        crate::blend::record_count("count.interference.revolution_declined", 1);
    }
    // Each sample's side is read off its nearest face where that read is
    // rigorous (`NearestFaceSide`), and cast as a ray otherwise: the same
    // verdict, without two agreeing rays per sample.
    let edge_build_timing = crate::blend::BlendTimer::start("timing.interference.edges.build_ms");
    let skin = match fresh.is_empty() {
        true => None,
        false => Some((
            SolidClassifier::new(original, policy.pcurve_consistency / 10.0)
                .or_refuse(KernelStage::Validate, "edge_classifier")?,
            NearestFaceSide::new(original, policy.pcurve_consistency / 10.0)
                .or_refuse(KernelStage::Validate, "edge_nearest_face")?,
        )),
    };
    edge_build_timing.stop();
    for edge in fresh {
        let Some((skin, nearest)) = skin.as_ref() else { break };
        let mut sampled = 0usize;
        let mut violations: Vec<Vec3> = Vec::new();
        crate::blend::record_count("count.interference.edges", 1);
        for step in 1..EDGE_SAMPLES {
            let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / EDGE_SAMPLES as f64;
            let Ok(point) = edge.curve.evaluate(t) else {
                continue;
            };
            sampled += 1;
            crate::blend::record_count("count.interference.edge_samples", 1);
            let class = match nearest.side(point) {
                Some(class) => class,
                None => {
                    let Ok(classification) = skin.classify(point) else {
                        // A classifier error at a sample counts as no violation
                        // (integrator finding 2026-10-17): counted so it is seen.
                        crate::blend::record_count("count.interference.classify_err", 1);
                        continue;
                    };
                    classification.class
                }
            };
            if class == forbidden {
                violations.push(point);
            }
        }
        if violations.len() >= 2 {
            return Err(report(
                format!("edge {}", edge.id),
                &violations,
                sampled,
                "along the grown",
            ));
        }
    }

    edges_timing.stop();
    crate::classification::classify_profile_report("blend.interference.edges");
    // Pass 2 — the new FACES.  Interior grid nodes only: the boundary of a
    // blend face IS the rail, already covered above and already On.  This one
    // classifies at `intersection_fit`: a wall sample is not a boundary point,
    // so the band only has to absorb the fitted surface's own accuracy.
    if grown.is_empty() {
        return Ok(());
    }
    let faces_timing = crate::blend::BlendTimer::start("timing.interference.faces_ms");
    crate::classification::classify_profile_begin();
    crate::classification::containment_profile_begin();
    let face_build_timing = crate::blend::BlendTimer::start("timing.interference.faces.build_ms");
    let classifier = SolidClassifier::new(original, policy.intersection_fit)
        .or_refuse(KernelStage::Validate, "face_classifier")?;
    let nearest = NearestFaceSide::new(original, policy.intersection_fit)
        .or_refuse(KernelStage::Validate, "face_nearest_face")?;
    face_build_timing.stop();
    const GRID: usize = 9;
    for face in grown {
        crate::blend::record_count("count.interference.faces", 1);
        let mut sampled = 0usize;
        let mut violations: Vec<Vec3> = Vec::new();
        // The lattice's interior nodes, the far ones answered by one query
        // (`face_grid_classes`).
        let band_at = |uv: Vec2| Ok(face_sample_uv_band(face, uv.x, uv.y, policy.model));
        for (uv, class) in face_grid_classes(face, GRID, &band_at) {
            let Some(Ok(PolygonClass::Inside)) = class else {
                continue;
            };
            let Ok(point) = face.surface.evaluate(uv.x, uv.y) else {
                continue;
            };
            sampled += 1;
            crate::blend::record_count("count.interference.face_samples", 1);
            let class = match nearest.side(point) {
                Some(class) => class,
                None => {
                    let Ok(classification) = classifier.classify(point) else {
                        crate::blend::record_count("count.interference.classify_err", 1);
                        continue;
                    };
                    classification.class
                }
            };
            if class == forbidden {
                violations.push(point);
            }
        }
        // Two independent interior points on the wrong side of the material is
        // a region, not sampling noise.
        if violations.len() < 2 {
            continue;
        }
        let label = face
            .name
            .clone()
            .unwrap_or_else(|| format!("face {}", face.id));
        return Err(report(label, &violations, sampled, "inside the grown face"));
    }
    faces_timing.stop();
    crate::classification::classify_profile_report("blend.interference.faces");
    crate::classification::containment_profile_report("blend.interference.faces");
    Ok(())
}

/// The interior nodes of a `grid × grid` lattice over a face's domain, each
/// with the face's own answer to whether it is inside the trims
/// (`parameter_point_in_face` at the uv band of a spatial `spatial` at that
/// node, [`face_sample_uv_band`]), in lattice order (u outer, v inner,
/// `1..grid` each). `None` is a node the kernel was NOT asked about because
/// it is FAR: outside the uv box of every loop's control points by more than
/// its own band. A pcurve lies in the hull of its control points (positive
/// weights), so far nodes are farther than their band from every trim
/// chord; the set of far nodes is kept inside ONE connected piece of the
/// domain minus that box (two pieces exist only when the box spans the
/// whole u or the whole v range, and then every far node must sit on the
/// same side), and a face's membership is constant on a connected set that
/// meets no trim. One far node, the farthest from the box, is asked for them
/// all: `Outside` stands for every far node; anything else, or any doubt
/// about the hull (a weight not positive, no far node, more than one piece),
/// asks about every node. On a corner patch (a spherical triangle bounded
/// by three arcs of 57 control points each) the sphere lane answers each
/// node in ~73 µs, 2 600 arcs walked six times, and 56 of 64 nodes are far;
/// on an extruded wall every node is ~2 µs and nothing is far.
pub(crate) fn face_grid_classes(
    face: &FaceRecord,
    grid: usize,
    band_at: &dyn Fn(Vec2) -> Result<f64, String>,
) -> Vec<(Vec2, Option<Result<PolygonClass, String>>)> {
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return Vec::new();
    };
    // A node whose band cannot be read carries that error as its answer.
    let nodes: Vec<(Vec2, Result<f64, String>)> = (1..grid)
        .flat_map(|iu| (1..grid).map(move |iv| (iu, iv)))
        .map(|(iu, iv)| {
            let uv = Vec2 {
                x: u0 + (u1 - u0) * iu as f64 / grid as f64,
                y: v0 + (v1 - v0) * iv as f64 / grid as f64,
            };
            (uv, band_at(uv))
        })
        .collect();
    let ask = |uv: Vec2, band: &Result<f64, String>| {
        crate::blend::record_count("count.grid.nodes_asked", 1);
        match band {
            Ok(band) => parameter_point_in_face(face, uv, *band),
            Err(error) => Err(error.clone()),
        }
    };
    let every = |nodes: &[(Vec2, Result<f64, String>)]| nodes.iter().map(|(uv, band)| (*uv, Some(ask(*uv, band)))).collect::<Vec<_>>();
    // The loops' uv box, from control points that are all positively weighted.
    let mut hull: Option<[f64; 4]> = None;
    for coedge in face.loops.iter().flat_map(|record| record.coedges.iter()) {
        for control in &coedge.pcurve.control_points {
            if !(control.w > 0.0) {
                return every(&nodes);
            }
            let (x, y) = (control.x / control.w, control.y / control.w);
            if !(x.is_finite() && y.is_finite()) {
                return every(&nodes);
            }
            hull = Some(match hull {
                None => [x, x, y, y],
                Some([a, b, c, d]) => [a.min(x), b.max(x), c.min(y), d.max(y)],
            });
        }
    }
    let Some([ua, ub, va, vb]) = hull else {
        return every(&nodes);
    };
    // How far a node stands outside the box, and on which side-strip it
    // lies when the box spans a whole range: (-1, 0, +1) in u and in v.
    let far = |uv: Vec2, band: &Result<f64, String>| -> Option<(f64, (i8, i8))> {
        let Ok(band) = band else { return None };
        let band = *band;
        let du = if uv.x < ua { ua - uv.x } else if uv.x > ub { uv.x - ub } else { 0.0 };
        let dv = if uv.y < va { va - uv.y } else if uv.y > vb { uv.y - vb } else { 0.0 };
        let distance = du.max(dv);
        if !(distance > band) {
            return None;
        }
        let side_u = if uv.x < ua { -1 } else if uv.x > ub { 1 } else { 0 };
        let side_v = if uv.y < va { -1 } else if uv.y > vb { 1 } else { 0 };
        Some((distance, (side_u, side_v)))
    };
    let spans_u = ua <= u0 && ub >= u1;
    let spans_v = va <= v0 && vb >= v1;
    let far_nodes: Vec<(usize, f64, (i8, i8))> = nodes
        .iter()
        .enumerate()
        .filter_map(|(index, (uv, band))| far(*uv, band).map(|(distance, sides)| (index, distance, sides)))
        .collect();
    crate::blend::record_count("count.grid.nodes_far", far_nodes.len() as u64);
    if far_nodes.is_empty() {
        return every(&nodes);
    }
    // One connected piece: with the box spanning the whole u range the
    // far nodes must all be below or all above it in v (and likewise for v).
    let one_piece = (!spans_u || far_nodes.iter().all(|(_, _, sides)| sides.1 == far_nodes[0].2 .1))
        && (!spans_v || far_nodes.iter().all(|(_, _, sides)| sides.0 == far_nodes[0].2 .0));
    if !one_piece {
        crate::blend::record_count("count.grid.far_split", 1);
        return every(&nodes);
    }
    let &(farthest, _, _) = far_nodes
        .iter()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a far node");
    let representative = ask(nodes[farthest].0, &nodes[farthest].1);
    if !matches!(representative, Ok(PolygonClass::Outside)) {
        crate::blend::record_count("count.grid.far_not_outside", 1);
        return every(&nodes);
    }
    let is_far: std::collections::HashSet<usize> = far_nodes.iter().map(|(index, _, _)| *index).collect();
    nodes
        .iter()
        .enumerate()
        .map(|(index, (uv, band))| {
            if index == farthest {
                (*uv, Some(Ok(PolygonClass::Outside)))
            } else if is_far.contains(&index) {
                (*uv, None)
            } else {
                (*uv, Some(ask(*uv, band)))
            }
        })
        .collect()
}

/// `BREP_NEAREST_FACE_ANALYTIC_ONLY=1`: the road until 2026-10-17, on which
/// a general (NURBS) face within reach of the point hands it back to the ray.
fn nearest_face_analytic_only() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var("BREP_NEAREST_FACE_ANALYTIC_ONLY").as_deref() == Ok("1"))
}

/// The side of a solid's boundary a point is on, read from the point's
/// NEAREST boundary point instead of a ray: a cheap answer the ray
/// classifier ([`SolidClassifier::classify`]) would also give, or `None`
/// when the read is not rigorous, in which case the caller casts the ray.
///
/// For a closed solid the open segment from a point `P` to its nearest
/// boundary point `Q` meets no boundary (a point of the boundary on it
/// would be nearer), so `P` lies on the side of `Q`'s own face: inside when
/// `(P − Q)` runs against the face's outward normal. The read is rigorous
/// when `Q` is known to be a nearest boundary point: it is the foot of `P`
/// on a face's carrier, that foot lies INSIDE the face's trims (so it is on
/// the face, and no point of that face is nearer), every face whose box
/// comes nearer than `Q` has been projected, and every face whose foot is
/// NOT inside its trims (whose nearest point is then somewhere on its
/// boundary, no nearer than its carrier) has a carrier no nearer than `Q`.
/// A `P` within the classifier's On band of `Q` is the classifier's to
/// read (it answers `On` with a normal); so is a `P` whose offset from `Q`
/// does not run along the normal (a foot that is not a foot). Carriers are
/// projected with the kernel's own projector: the closed form on a plane,
/// a cylinder, a cone, a sphere or a torus, and on a general surface the
/// seed-grid Newton followed by the global stage that certifies the foot
/// against every lattice cell of the net (`project_point_to_surface_general`
/// states the one step it does not certify) — the same query every other
/// nearest-point read in the kernel rests on.
///
/// Built for the blend acceptance's interference scan and the restriction's
/// convexity vote, whose probes sit between the mates of a blend: a wall
/// sample's nearest boundary point is on a mate, and the ray classifier
/// spent 18–36 µs per sample confirming it with two agreeing rays.
pub(crate) struct NearestFaceSide<'a> {
    solid: &'a BrepSolid,
    faces: Vec<&'a FaceRecord>,
    boxes: Vec<crate::Aabb>,
    bounds: crate::Aabb,
    on_band: f64,
}

impl<'a> NearestFaceSide<'a> {
    /// `tolerance` is the ray classifier's: its On band is ten times it.
    pub(crate) fn new(solid: &'a BrepSolid, tolerance: f64) -> Result<Self, String> {
        let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
        let boxes = faces.iter().map(|face| crate::Aabb::from_surface_controls(&face.surface)).collect::<Result<Vec<_>, _>>()?;
        let mut bounds = crate::Aabb::empty();
        for face_box in &boxes {
            bounds.include(*face_box);
        }
        Ok(Self { solid, faces, boxes, bounds, on_band: tolerance * 10.0 })
    }

    /// Distance from `point` to a box (zero inside it).
    fn box_distance(face_box: &crate::Aabb, point: Vec3) -> f64 {
        let axis = |value: f64, low: f64, high: f64| if value < low { low - value } else if value > high { value - high } else { 0.0 };
        let dx = axis(point.x, face_box.minimum.x, face_box.maximum.x);
        let dy = axis(point.y, face_box.minimum.y, face_box.maximum.y);
        let dz = axis(point.z, face_box.minimum.z, face_box.maximum.z);
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// `Some(In)` or `Some(Out)` when the nearest-face read is rigorous
    /// (above), `None` otherwise.
    pub(crate) fn side(&self, point: Vec3) -> Option<PointClass> {
        match self.side_or_reason(point) {
            Ok(class) => Some(class),
            Err(reason) => {
                // Why a sample went to the ray, under `BREP_DEBUG_NEAREST_SIDE`
                // (2026-10-17 face-ring timing: every wall-interior sample of
                // the interference scan was cast as two rays).
                if std::env::var_os("BREP_DEBUG_NEAREST_SIDE").is_some() {
                    eprintln!("nearest-face side: ray for ({:.6}, {:.6}, {:.6}): {reason}", point.x, point.y, point.z);
                }
                crate::blend::record_count(reason_counter(reason), 1);
                None
            }
        }
    }

    /// [`Self::side`], naming the condition that hands the point to the ray.
    fn side_or_reason(&self, point: Vec3) -> Result<PointClass, &'static str> {
        if !self.bounds.expanded(self.on_band).contains(point) {
            // The classifier's own first answer (Out), left to it.
            return Err("outside the body's box");
        }
        let mut order: Vec<(f64, usize)> = self.boxes.iter().enumerate().map(|(index, face_box)| (Self::box_distance(face_box, point), index)).collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0));
        // The nearest interior foot, and the nearest carrier among faces
        // whose foot was not inside their trims.
        let mut nearest: Option<(f64, usize, SurfaceProjection)> = None;
        // Faces whose foot fell outside their trims, with their carrier distance:
        // their nearest point is on their boundary, no nearer than the carrier.
        let mut exterior: Vec<(f64, usize)> = Vec::new();
        for &(lower_bound, index) in &order {
            if nearest.as_ref().is_some_and(|(distance, _, _)| lower_bound >= *distance) {
                break;
            }
            let face = self.faces[index];
            if nearest_face_analytic_only() && face.surface.analytic().is_none() {
                return Err("a general carrier within reach (analytic-only switch)");
            }
            // The kernel's own nearest-point query: the closed form on an
            // analytic carrier, the certified global stage on a general
            // surface (`projection.rs::project_point_to_surface_general`,
            // which says what it guarantees). Until 2026-10-17 a general
            // face within reach handed the point back to the ray instead,
            // and on a body whose blends are fitted surfaces that was
            // every point: the 2026-10-17 shelled-box report's F9 cast
            // 216 rays for its 108 wall samples at 6.4 ms a sample — each
            // ray clipped to ten faces' boxes and seeded 32 Newtons on
            // each — where this read answers from two or three feet at
            // 30–300 µs each.
            let projection = project_point_to_surface(&face.surface, point).map_err(|_| "a carrier projection failed")?;
            if !projection.distance.is_finite() {
                return Err("a carrier projection read a non-finite distance");
            }
            let band = crate::classification::surface_uv_band(&face.surface, projection.u, projection.v, self.on_band);
            let inside = parameter_point_in_face(face, Vec2 { x: projection.u, y: projection.v }, band).map_err(|_| "a trim test failed")?;
            if inside == PolygonClass::Inside {
                if nearest.as_ref().map_or(true, |(distance, _, _)| projection.distance < *distance) {
                    nearest = Some((projection.distance, index, projection));
                }
            } else {
                exterior.push((projection.distance, index));
            }
        }
        let (distance, index, projection) = nearest.ok_or("no foot inside a face's trims")?;
        if !(distance > self.on_band) {
            // The classifier's own On band is this band (`SolidClassifier::classify`
            // reads On at any face within `tolerance * 10` whose foot is inside its
            // trims), so its answer here is On too; said directly, the rail
            // samples of the interference scan — every one ON a mate by
            // construction — no longer cast a ray each (2026-10-17 face ring:
            // 531 of its 861 rays).
            return Ok(PointClass::On);
        }
        for &(carrier_distance, exterior_index) in &exterior {
            if carrier_distance >= distance {
                continue;
            }
            // That face's nearest point is on its boundary: no nearer than the
            // point's distance to its boundary edges' curves (the whole curve, a
            // bound no larger than the edge's own). Nearer than `Q`, and the
            // read is not rigorous; otherwise `Q` stays the nearest.
            let face = self.faces[exterior_index];
            let mut boundary = f64::INFINITY;
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    let Some(edge) = self.solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                        return Err("a boundary edge is missing");
                    };
                    // The edge's own range of its curve where it can be cut out
                    // (`split` at both ends); the whole curve, a bound no larger
                    // than the edge's, where it cannot.
                    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
                    let piece = edge
                        .curve
                        .split(low)
                        .and_then(|(_, rest)| rest.split(high))
                        .map(|(piece, _)| piece)
                        .unwrap_or_else(|_| edge.curve.clone());
                    let projection = crate::project_point_to_curve(&piece, point).map_err(|_| "a boundary curve projection failed")?;
                    if !projection.distance.is_finite() {
                        return Err("a boundary curve projection read a non-finite distance");
                    }
                    boundary = boundary.min(projection.distance);
                }
            }
            if !(boundary > self.on_band) {
                // The point is within the On band of that face's boundary, so of
                // the face: the classifier reads On there.
                return Ok(PointClass::On);
            }
            if !(boundary > distance) {
                if std::env::var_os("BREP_DEBUG_NEAREST_SIDE").is_some() {
                    eprintln!(
                        "nearest-face side: exterior face {} ({}) carrier {:.3e} boundary {:.3e} vs nearest face {} ({}) at {:.3e}",
                        face.id, face.name.as_deref().unwrap_or("-"), carrier_distance, boundary,
                        self.faces[index].id, self.faces[index].name.as_deref().unwrap_or("-"), distance
                    );
                }
                return Err("an exterior foot's carrier is nearer than the nearest face point");
            }
        }
        let face = self.faces[index];
        let normal = face.surface.normal(projection.u, projection.v).map_err(|_| "the normal failed")?;
        let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
        let along = point.sub(projection.point).dot(normal);
        if !(along.abs() >= 0.5 * distance) {
            return Err("the offset does not run along the normal");
        }
        Ok(if along > 0.0 { PointClass::Out } else { PointClass::In })
    }
}

/// The ledger counter for a [`NearestFaceSide::side`] fall-through reason.
fn reason_counter(reason: &'static str) -> &'static str {
    match reason {
        "outside the body's box" => "count.nearest_side.ray.outside_box",
        "no foot inside a face's trims" => "count.nearest_side.ray.no_interior_foot",
        "within the On band" => "count.nearest_side.ray.on_band",
        "an exterior foot's carrier is nearer than the nearest face point" => "count.nearest_side.ray.exterior_nearer",
        "the offset does not run along the normal" => "count.nearest_side.ray.off_normal",
        _ => "count.nearest_side.ray.failed_query",
    }
}

/// UV band equivalent to a spatial band of `spatial` at (u, v), capped to a
/// fraction of the smaller domain span so a pole cannot widen it into the whole
/// face — the same derivation `SolidClassifier` uses for its own trim probes.
pub(crate) fn face_sample_uv_band(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> f64 {
    let Ok(derivatives) = face.surface.derivatives(u, v, 1) else {
        return spatial;
    };
    let band = crate::tolerance::surface_uv_tolerance(
        spatial,
        derivatives[1][0].length(),
        derivatives[0][1].length(),
    );
    let cap = match (face.surface.domain_u(), face.surface.domain_v()) {
        (Ok([a0, a1]), Ok([b0, b1])) => ((a1 - a0).min(b1 - b0) * 0.05).max(1e-12),
        _ => f64::INFINITY,
    };
    band.min(cap)
}

thread_local! {
    /// Faces whose loops [`crate::loop_self_crossings`] read CLEAN (no
    /// crossing, nothing unreadable), keyed by the face's serialised record
    /// and the bits of the solid scale its vertex band is derived from.
    pub(super) static CLEAN_LOOP_FACES: std::cell::RefCell<std::collections::HashSet<Vec<u8>>> =
        std::cell::RefCell::new(std::collections::HashSet::new());
}

/// Bound on [`CLEAN_LOOP_FACES`]; the set is emptied when it would pass it.
const CLEAN_LOOP_FACES_CAP: usize = 8192;

/// [`crate::loop_self_crossings`] of `result`, every face it already read clean
/// on an earlier call skipped. The scan reads each face on its own -- its
/// surface, its loops' pcurves and edge ids, its id and name -- and one
/// solid-wide band, `assembler_weld` of the model scale of the solid's
/// VERTICES. So a copy holding every vertex but only the faces not yet read
/// reports, face for face and in the same order, what the whole scan reports
/// for those faces, and a face whose serialised record and scale are those of
/// a face read clean before reads clean again. The acceptance reads only the
/// crossings (the first and the count), which are therefore the whole scan's.
/// A composition re-scanned the whole evolving body at every component: 13
/// scans, 8.4 s of the sprocket report's 68.5 s `F9` (2026-10-14).
/// `BREP_BLEND_BOWTIE_FACE_MEMO=0` scans every face every time, as before.
pub(super) fn loop_self_crossings_reusing_clean_faces(result: &BrepSolid) -> crate::LoopCrossingReport {
    if std::env::var("BREP_BLEND_BOWTIE_FACE_MEMO").as_deref() == Ok("0") {
        return crate::loop_self_crossings(result);
    }
    let scale = crate::tolerance::solid_scale(result).to_bits().to_le_bytes();
    let key = |face: &crate::topology::FaceRecord| -> Option<Vec<u8>> {
        let mut bytes = serde_json::to_vec(face).ok()?;
        bytes.extend_from_slice(&scale);
        Some(bytes)
    };
    let mut reduced = result.clone();
    let mut pending: Vec<(u64, Option<Vec<u8>>)> = Vec::new();
    let mut skipped = 0u64;
    CLEAN_LOOP_FACES.with(|clean| {
        let clean = clean.borrow();
        for shell in &mut reduced.shells {
            shell.faces.retain(|face| {
                let key = key(face);
                if key.as_ref().is_some_and(|key| clean.contains(key)) {
                    skipped += 1;
                    return false;
                }
                pending.push((face.id, key));
                true
            });
        }
    });
    crate::blend::record_count("count.bowtie.faces_reused", skipped);
    crate::blend::record_count("count.bowtie.faces_scanned", pending.len() as u64);
    let report = crate::loop_self_crossings(&reduced);
    if report.unreadable.is_empty() {
        CLEAN_LOOP_FACES.with(|clean| {
            let mut clean = clean.borrow_mut();
            for (id, key) in pending {
                let Some(key) = key else { continue };
                if report.crossings.iter().any(|crossing| crossing.face == id) {
                    continue;
                }
                if clean.len() >= CLEAN_LOOP_FACES_CAP {
                    clean.clear();
                }
                clean.insert(key);
            }
        });
    }
    report
}


/// **Does any face of the result have a trim loop that crosses ITSELF?**
///
/// The acceptance's other three questions — the heal, `validate()` and
/// [`check_blend_interference`] — all pass a bowtie. A face whose outer loop
/// crosses itself is perfectly incident (every coedge chains, every edge is
/// used twice with opposite senses), it is connected, its Euler parity is
/// unchanged, and it crosses no OTHER face, so the face-versus-face scan has
/// no pair to confirm. The 2026-09-13 rib-spine partition composed exactly
/// that: both rib side faces came back with a loop crossing itself twice, and
/// every detector in the kernel passed the solid.
///
/// What it means geometrically is that a blend wall's contact rail ran off the
/// end of its own face and the surgery spliced the whole rail into the loop
/// instead of clipping it to where the face stops. The overhang comes back as
/// an inverted lobe: the enclosed VOLUME still comes out right (an inverted
/// lobe's flux equals a correctly oriented one's), and the face's AREA is
/// short by exactly the overhang — which is why nothing that measures volume
/// notices.
///
/// So this is a refusal, not a repair: the shape asked for is a trim this lane
/// does not construct, and returning the bowtie reports success for a
/// different result. The message names the face and the two edges, because the
/// question a caller then has is WHICH wall ran off WHICH face.
pub(super) fn check_loop_self_crossings(
    result: &BrepSolid,
    entry: &str,
) -> Result<(), KernelRefusal> {
    let _timing = crate::blend::BlendTimer::start("timing.bowtie_ms");
    let report = loop_self_crossings_reusing_clean_faces(result);
    let Some(crossing) = report.crossings.first() else {
        return Ok(());
    };
    // The runout trim that would clip the rail to its face is a construction
    // this lane does not perform (see the header): a named deferral.
    Err(KernelRefusal::unsupported(
        KernelStage::Validate,
        "loop_self_crossing",
        format!(
            "{entry}: the result's face {}{} has a loop that CROSSES ITSELF at \
             ({:.6}, {:.6}, {:.6}) — its edges {} and {} cross in the face's own parameter \
             domain{}. A blend wall whose contact rail runs off the end of its own face is not \
             trimmed to it, and the face that comes back is wrong in AREA where its volume is \
             not; `validate()` cannot see it. Blend the edges that need the runout separately.",
            crossing.face,
            crossing
                .face_name
                .as_ref()
                .map(|name| format!(" '{name}'"))
                .unwrap_or_default(),
            crossing.point.x,
            crossing.point.y,
            crossing.point.z,
            crossing.edge_a,
            crossing.edge_b,
            if report.crossings.len() > 1 {
                format!(" ({} crossings in all)", report.crossings.len())
            } else {
                String::new()
            },
        ),
    ))
}

/// Samples along an iso-line and the window the reversal test reads across.
const WALL_LOOP_SAMPLES: usize = 512;
const WALL_LOOP_LINES: usize = 9;
const WALL_LOOP_WINDOW: usize = 8;

thread_local! {
    /// Faces whose iso-lines read free of a loop, keyed by the face's content
    /// digest (`hash_face`) and the scale bits the bar is derived from.
    static LOOP_FREE_FACES: std::cell::RefCell<rustc_hash::FxHashSet<[u64; 2]>> =
        std::cell::RefCell::new(rustc_hash::FxHashSet::default());
}

/// Bound on [`LOOP_FREE_FACES`]; the set is emptied when it would pass it.
const LOOP_FREE_FACES_CAP: usize = 8192;

/// Does any fitted face of `result` pass back through itself along an
/// iso-line? (2026-10-17, `fillet-mouth-over-wall-bore-5p5-r2p2`: the F2
/// wall's row at v = 0.156 runs x = 10.000 → 9.974 → 10.000 → 10.026 →
/// 10.000 across the rim's midpoint, a 2.6e-2-wide loop the soundness
/// scan's fold half — two of the face's own mesh triangles crossing — never
/// saw under the 512-sample rim mesh and convicted under a 40-sample one.
/// The verdict must not depend on a mesh.)
///
/// Each general (non-analytic) face is read along [`WALL_LOOP_LINES`]
/// iso-lines in each direction at [`WALL_LOOP_SAMPLES`] points: a step
/// between consecutive samples that runs AGAINST the line's own direction
/// over the [`WALL_LOOP_WINDOW`] samples either side of it is the line
/// doubling back — a loop at least a window wide, which at these counts is
/// 3 % of the domain, two station intervals of a 64-station march, and far
/// below any bend a wall can legitimately turn through in that span. Steps
/// and windows shorter than the bar (`1e-7 (1 + scale)`, the fit's own
/// residual three decades down) are not read, so a pole or a collapsed
/// section flags nothing. A face read clean is not read again
/// (`LOOP_FREE_FACES`, keyed on its content). The refusal is the wall-fold
/// class (`blend::wall_loop_refusal`), terminal as the march's own fold
/// verdicts are. `BREP_BLEND_WALL_LOOP_CHECK=0` is the hatch.
pub(super) fn check_blend_wall_loops(result: &BrepSolid, entry: &str) -> Result<(), KernelRefusal> {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if !*ENABLED.get_or_init(|| std::env::var("BREP_BLEND_WALL_LOOP_CHECK").as_deref() != Ok("0")) {
        return Ok(());
    }
    let _timing = crate::blend::BlendTimer::start("timing.wall_loops_ms");
    let scale = crate::solid_scale(result);
    let bar = 1.0e-7 * (1.0 + scale);
    for face in result.shells.iter().flat_map(|shell| shell.faces.iter()) {
        if face.surface.analytic().is_some() {
            continue;
        }
        let key = {
            let mut lane = crate::mass_properties::TwoLane::new();
            crate::mass_properties::hash_face(&mut lane, face);
            lane.f64(scale);
            lane.finish()
        };
        if LOOP_FREE_FACES.with(|set| set.borrow().contains(&key)) {
            continue;
        }
        crate::blend::record_count("count.wall_loops.faces_read", 1);
        let Ok([u0, u1]) = face.surface.domain_u() else { continue };
        let Ok([v0, v1]) = face.surface.domain_v() else { continue };
        for (along_u, lines_from, lines_to, from, to) in [(true, v0, v1, u0, u1), (false, u0, u1, v0, v1)] {
            for l in 0..WALL_LOOP_LINES {
                let line = lines_from + (lines_to - lines_from) * (l as f64 + 0.5) / WALL_LOOP_LINES as f64;
                let mut points = Vec::with_capacity(WALL_LOOP_SAMPLES + 1);
                for k in 0..=WALL_LOOP_SAMPLES {
                    let t = from + (to - from) * k as f64 / WALL_LOOP_SAMPLES as f64;
                    let point = if along_u { face.surface.evaluate(t, line) } else { face.surface.evaluate(line, t) };
                    let Ok(point) = point else { break };
                    points.push((t, point));
                }
                if points.len() != WALL_LOOP_SAMPLES + 1 {
                    continue;
                }
                for k in WALL_LOOP_WINDOW..WALL_LOOP_SAMPLES - WALL_LOOP_WINDOW {
                    let step = points[k + 1].1.sub(points[k].1);
                    let window = points[k + WALL_LOOP_WINDOW].1.sub(points[k - WALL_LOOP_WINDOW].1);
                    let (step_length, window_length) = (step.length(), window.length());
                    if step_length <= bar || window_length <= bar {
                        continue;
                    }
                    if step.dot(window) < -bar * window_length {
                        // Only MATERIAL can fold: a reversal whose samples lie
                        // outside the face's trim is the untrimmed carrier
                        // doubling back where the face does not reach — the
                        // lens a carved wall cut away (the closed-edge lane's
                        // carve leaves the crease as a zero-width slit, and
                        // the surface inside it still loops: 2026-10-17,
                        // `fillet-mouth-over-wall-bore-5p5-r2p2`). Both
                        // samples of the step must be strictly inside the
                        // trim; a sample on the trim boundary is the slit's.
                        let uv_band = crate::tolerance::surface_uv_tolerance(bar, 1.0, 1.0).max(1e-9);
                        let uv_of = |t: f64| if along_u { Vec2 { x: t, y: line } } else { Vec2 { x: line, y: t } };
                        let material = [points[k].0, points[k + 1].0].iter().all(|&t| {
                            matches!(parameter_point_in_face(face, uv_of(t), uv_band), Ok(PolygonClass::Inside))
                        });
                        if !material {
                            continue;
                        }
                        let (which, other) = if along_u { ("row (u)", "v") } else { ("column (v)", "u") };
                        return Err(crate::blend::wall_loop_refusal(
                            face.id,
                            format!(
                                "{} {entry}: the result's face {}{} passes back through itself along its {which} \
                                 line at {other} = {line:.6}: between parameters {:.6} and {:.6} the line moves \
                                 {step_length:.3e} against its own direction over the surrounding window \
                                 ({:.6}..{:.6}), so the wall carries two parameters on one point there. A \
                                 fitted wall that loops is wrong in shape, not in area; nothing downstream can \
                                 trim it right.",
                                crate::blend::WALL_FOLDS,
                                face.id,
                                face.name.as_ref().map(|name| format!(" '{name}'")).unwrap_or_default(),
                                points[k].0,
                                points[k + 1].0,
                                points[k - WALL_LOOP_WINDOW].0,
                                points[k + WALL_LOOP_WINDOW].0,
                            ),
                        ));
                    }
                }
            }
        }
        LOOP_FREE_FACES.with(|set| {
            let mut set = set.borrow_mut();
            if set.len() >= LOOP_FREE_FACES_CAP {
                set.clear();
            }
            set.insert(key);
        });
    }
    Ok(())
}

