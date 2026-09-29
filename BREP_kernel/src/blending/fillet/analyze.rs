use super::*;

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

pub(super) fn face_plane_normal(
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
fn neighbour_carries_ball(
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
    let Ok(cross) = analyze_edge(solid, edge_id, size) else {
        return Ok(());
    };
    let Ok((_, faces)) = edge_and_faces(solid, edge_id) else {
        return Ok(());
    };
    if faces.len() != 2 {
        return Ok(());
    }
    let phi = cross
        .into_first
        .dot(cross.into_second)
        .clamp(-1.0, 1.0)
        .acos();
    // Tangent supports are already refused by `analyze_edge`; a degenerate
    // wedge here would only make `reach` meaningless.
    if !(phi > 1e-9) || phi >= std::f64::consts::PI - 1e-9 {
        return Ok(());
    }
    let reach = size / (phi * 0.5).tan();
    if !reach.is_finite() || reach <= 0.0 {
        return Ok(());
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
    for (face, into) in [
        (faces[0], cross.into_first),
        (faces[1], cross.into_second),
    ] {
        // The face's own boundary, in 3D.  The test below is a BOUND, not a
        // containment test, and deliberately so: `project_point_to_surface`
        // clamps to the carrier's parameter rectangle, so a contact point past
        // a box face's own patch comes back AT the boundary with a residual —
        // exactly the census case — and `parameter_point_in_face` then
        // classifies the clamped point Inside.  Measuring how far the face
        // itself reaches along the into-direction needs no inversion, and it
        // errs only toward NOT refusing: a face that reaches far enough
        // somewhere is given the benefit of the doubt even if it does not
        // reach there at this station.
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
        let sample_face = |sampled: &FaceRecord, boundary: &mut Vec<Vec3>| {
            for loop_record in &sampled.loops {
                for coedge in &loop_record.coedges {
                    let Some(border) = solid.edges.iter().find(|e| e.id == coedge.edge_id) else {
                        continue;
                    };
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
                EdgePath::Straight { direction, length } if straight => (
                    cross.start.add(direction.scale(length * fraction)),
                    into,
                ),
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
            return Err(KernelRefusal::input(
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
            ));
        }
    }
    Ok(())
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
    use crate::{parameter_point_in_face, PointClass, PolygonClass, SolidClassifier, Vec2};

    if edge_ids.is_empty() || !original.validate().is_empty() {
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
    let inherited: Vec<u64> = original.edges.iter().map(|edge| edge.id).collect();
    let fresh: Vec<_> = result
        .edges
        .iter()
        .filter(|edge| !inherited.contains(&edge.id))
        .collect();
    let skin = match fresh.is_empty() {
        true => None,
        false => Some(
            SolidClassifier::new(original, policy.pcurve_consistency / 10.0)
                .or_refuse(KernelStage::Validate, "edge_classifier")?,
        ),
    };
    for edge in fresh {
        let Some(skin) = skin.as_ref() else { break };
        let mut sampled = 0usize;
        let mut violations: Vec<Vec3> = Vec::new();
        for step in 1..EDGE_SAMPLES {
            let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / EDGE_SAMPLES as f64;
            let Ok(point) = edge.curve.evaluate(t) else {
                continue;
            };
            sampled += 1;
            let Ok(classification) = skin.classify(point) else {
                continue;
            };
            if classification.class == forbidden {
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

    // Pass 2 — the new FACES.  Interior grid nodes only: the boundary of a
    // blend face IS the rail, already covered above and already On.  This one
    // classifies at `intersection_fit`: a wall sample is not a boundary point,
    // so the band only has to absorb the fitted surface's own accuracy.
    if grown.is_empty() {
        return Ok(());
    }
    let classifier = SolidClassifier::new(original, policy.intersection_fit)
        .or_refuse(KernelStage::Validate, "face_classifier")?;
    const GRID: usize = 9;
    for face in grown {
        let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v())
        else {
            continue;
        };
        let mut sampled = 0usize;
        let mut violations: Vec<Vec3> = Vec::new();
        for iu in 1..GRID {
            for iv in 1..GRID {
                let uv = Vec2 {
                    x: u0 + (u1 - u0) * iu as f64 / GRID as f64,
                    y: v0 + (v1 - v0) * iv as f64 / GRID as f64,
                };
                let band = face_sample_uv_band(face, uv.x, uv.y, policy.model);
                let Ok(PolygonClass::Inside) = parameter_point_in_face(face, uv, band) else {
                    continue;
                };
                let Ok(point) = face.surface.evaluate(uv.x, uv.y) else {
                    continue;
                };
                sampled += 1;
                let Ok(classification) = classifier.classify(point) else {
                    continue;
                };
                if classification.class == forbidden {
                    violations.push(point);
                }
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
    Ok(())
}

/// UV band equivalent to a spatial band of `spatial` at (u, v), capped to a
/// fraction of the smaller domain span so a pole cannot widen it into the whole
/// face — the same derivation `SolidClassifier` uses for its own trim probes.
fn face_sample_uv_band(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> f64 {
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
    let report = crate::loop_self_crossings(result);
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
