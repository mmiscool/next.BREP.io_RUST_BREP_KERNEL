use super::*;

// ---------------------------------------------------------------------------
// Mate expansion + validation
// ---------------------------------------------------------------------------

pub(super) fn mate_name(index: usize, mate: &AssemblyMate) -> String {
    mate.id.clone().unwrap_or_else(|| format!("mate[{index}]"))
}

pub(super) fn vec3(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0], v[1], v[2])
}

fn finite_point(v: [f64; 3], what: &str, mate: &str) -> Result<Vec3, String> {
    if !(v[0].is_finite() && v[1].is_finite() && v[2].is_finite()) {
        return Err(format!("{mate}: {what} must be finite"));
    }
    Ok(vec3(v))
}

fn unit_dir(v: [f64; 3], what: &str, mate: &str) -> Result<Vec3, String> {
    let v = finite_point(v, what, mate)?;
    let len = v.length();
    if !(len > ASM_DIR_EPS) {
        return Err(format!("{mate}: {what} must be a nonzero vector"));
    }
    Ok(v.scale(1.0 / len))
}

fn finite_scalar(x: f64, what: &str, mate: &str) -> Result<f64, String> {
    if !x.is_finite() {
        return Err(format!("{mate}: {what} must be finite"));
    }
    Ok(x)
}

/// Direction-sense atom for `align` between two unit local directions.
fn dir_atom(a: LocalDir, b: LocalDir, align: MateAlign) -> Atom {
    match align {
        MateAlign::Aligned => Atom::DirMatch(a, b, 1.0),
        MateAlign::AntiAligned => Atom::DirMatch(a, b, -1.0),
        MateAlign::Any => Atom::DirCross(a, b),
    }
}

/// Expand every mate into residual atoms. Returns the atoms plus a parallel
/// map atom → mate index for residual attribution.
pub(super) fn expand_mates(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
) -> Result<(Vec<Atom>, Vec<usize>), String> {
    let mut atoms: Vec<Atom> = Vec::new();
    let mut atom_mate: Vec<usize> = Vec::new();
    for (mi, mate) in mates.iter().enumerate() {
        let name = mate_name(mi, mate);
        if mate.body_a >= bodies.len() || mate.body_b >= bodies.len() {
            return Err(format!(
                "{name}: body index out of range (body_a={}, body_b={}, {} bodies)",
                mate.body_a,
                mate.body_b,
                bodies.len()
            ));
        }
        if mate.body_a == mate.body_b {
            return Err(format!("{name}: body_a and body_b must differ"));
        }
        if let Some(c) = mate.body_c {
            if c >= bodies.len() {
                return Err(format!("{name}: body_c out of range ({c}, {} bodies)", bodies.len()));
            }
        }
        let ba = mate.body_a;
        let bb = mate.body_b;
        let before = atoms.len();
        match &mate.kind {
            MateKind::CoincidentPointPoint { point_a, point_b } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let pb = finite_point(*point_b, "point_b", &name)?;
                atoms.push(Atom::PointsCoincide(
                    LocalPoint { body: ba, p: pa },
                    LocalPoint { body: bb, p: pb },
                ));
            }
            MateKind::CoincidentPointPlane { point_a, plane_b } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let po = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let pn = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: ba, p: pa },
                    LocalPoint { body: bb, p: po },
                    LocalDir { body: bb, d: pn },
                    0.0,
                ));
            }
            MateKind::CoincidentPlanePlane {
                plane_a,
                plane_b,
                align,
            } => {
                let oa = finite_point(plane_a.origin, "plane_a.origin", &name)?;
                let na = unit_dir(plane_a.normal, "plane_a.normal", &name)?;
                let ob = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let nb = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                atoms.push(dir_atom(
                    LocalDir { body: ba, d: na },
                    LocalDir { body: bb, d: nb },
                    *align,
                ));
                // Plane B's origin lies on plane A.
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: bb, p: ob },
                    LocalPoint { body: ba, p: oa },
                    LocalDir { body: ba, d: na },
                    0.0,
                ));
            }
            MateKind::ConcentricAxisAxis {
                axis_a,
                axis_b,
                align,
            } => {
                let oa = finite_point(axis_a.origin, "axis_a.origin", &name)?;
                let da = unit_dir(axis_a.direction, "axis_a.direction", &name)?;
                let ob = finite_point(axis_b.origin, "axis_b.origin", &name)?;
                let db = unit_dir(axis_b.direction, "axis_b.direction", &name)?;
                atoms.push(dir_atom(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: db },
                    *align,
                ));
                // Axis A's origin lies on axis B's line (lateral offset zero).
                atoms.push(Atom::AxisLateral(
                    LocalPoint { body: ba, p: oa },
                    LocalPoint { body: bb, p: ob },
                    LocalDir { body: bb, d: db },
                ));
            }
            MateKind::DistancePointPoint {
                point_a,
                point_b,
                distance,
            } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let pb = finite_point(*point_b, "point_b", &name)?;
                let d = finite_scalar(*distance, "distance", &name)?;
                let lp_a = LocalPoint { body: ba, p: pa };
                let lp_b = LocalPoint { body: bb, p: pb };
                if d.abs() <= ASM_DIR_EPS {
                    // ‖·‖ is non-differentiable at zero: degrade to coincident.
                    atoms.push(Atom::PointsCoincide(lp_a, lp_b));
                } else {
                    atoms.push(Atom::PointDistance(lp_a, lp_b, d.abs()));
                }
            }
            MateKind::DistancePointPlane {
                point_a,
                plane_b,
                distance,
            } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let po = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let pn = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                let d = finite_scalar(*distance, "distance", &name)?;
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: ba, p: pa },
                    LocalPoint { body: bb, p: po },
                    LocalDir { body: bb, d: pn },
                    d,
                ));
            }
            MateKind::DistancePlanePlane {
                plane_a,
                plane_b,
                distance,
                align,
            } => {
                let oa = finite_point(plane_a.origin, "plane_a.origin", &name)?;
                let na = unit_dir(plane_a.normal, "plane_a.normal", &name)?;
                let ob = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let nb = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                let d = finite_scalar(*distance, "distance", &name)?;
                atoms.push(dir_atom(
                    LocalDir { body: ba, d: na },
                    LocalDir { body: bb, d: nb },
                    *align,
                ));
                // Signed offset of plane B's origin along plane A's normal.
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: bb, p: ob },
                    LocalPoint { body: ba, p: oa },
                    LocalDir { body: ba, d: na },
                    d,
                ));
            }
            MateKind::DistancePointLine {
                point_a,
                axis_b,
                distance,
            } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let ob = finite_point(axis_b.origin, "axis_b.origin", &name)?;
                let db = unit_dir(axis_b.direction, "axis_b.direction", &name)?;
                let d = finite_scalar(*distance, "distance", &name)?;
                let lp = LocalPoint { body: ba, p: pa };
                let lo = LocalPoint { body: bb, p: ob };
                let ld = LocalDir { body: bb, d: db };
                if d.abs() <= ASM_DIR_EPS {
                    // ‖·‖ is non-differentiable at zero: degrade to the
                    // point-on-line lateral atom (the same one concentric
                    // uses) — the coincident point-line shortcut.
                    atoms.push(Atom::AxisLateral(lp, lo, ld));
                } else {
                    atoms.push(Atom::PointLineDistance(lp, lo, ld, d.abs()));
                }
            }
            MateKind::DistanceLineLine {
                axis_a,
                axis_b,
                distance,
            } => {
                let oa = finite_point(axis_a.origin, "axis_a.origin", &name)?;
                let da = unit_dir(axis_a.direction, "axis_a.direction", &name)?;
                let ob = finite_point(axis_b.origin, "axis_b.origin", &name)?;
                let db = unit_dir(axis_b.direction, "axis_b.direction", &name)?;
                let d = finite_scalar(*distance, "distance", &name)?;
                let loa = LocalPoint { body: ba, p: oa };
                let lda = LocalDir { body: ba, d: da };
                let lob = LocalPoint { body: bb, p: ob };
                let ldb = LocalDir { body: bb, d: db };
                if d.abs() <= ASM_DIR_EPS {
                    // Distance 0 = the lines touch: intersect when skew,
                    // collinear when parallel. The signed vector atom avoids
                    // the ‖·‖ − 0 kink at the solution.
                    atoms.push(Atom::LineLineTouch(loa, lda, lob, ldb));
                } else {
                    atoms.push(Atom::LineLineDistance(loa, lda, lob, ldb, d.abs()));
                }
            }
            MateKind::Angle {
                direction_a,
                direction_b,
                angle_deg,
            } => {
                let da = unit_dir(*direction_a, "direction_a", &name)?;
                let db = unit_dir(*direction_b, "direction_b", &name)?;
                let ang = finite_scalar(*angle_deg, "angle_deg", &name)?;
                atoms.push(Atom::DirDot(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: db },
                    ang.to_radians().cos(),
                ));
            }
            MateKind::DirectedAngle {
                direction_a,
                direction_b,
                axis_a,
                angle_deg,
            } => {
                let da = unit_dir(*direction_a, "direction_a", &name)?;
                let db = unit_dir(*direction_b, "direction_b", &name)?;
                let axis = unit_dir(*axis_a, "axis_a", &name)?;
                if da.dot(axis).abs() > 1e-8 {
                    return Err(format!("{name}: axis_a must be perpendicular to direction_a"));
                }
                let ang = finite_scalar(*angle_deg, "angle_deg", &name)?;
                atoms.push(Atom::DirAngle(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: db },
                    LocalDir { body: ba, d: axis },
                    ang.rem_euclid(360.0).to_radians(),
                ));
            }
            MateKind::Parallel {
                direction_a,
                direction_b,
                align,
            } => {
                let da = unit_dir(*direction_a, "direction_a", &name)?;
                let db = unit_dir(*direction_b, "direction_b", &name)?;
                atoms.push(dir_atom(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: db },
                    *align,
                ));
            }
            MateKind::Perpendicular {
                direction_a,
                direction_b,
            } => {
                let da = unit_dir(*direction_a, "direction_a", &name)?;
                let db = unit_dir(*direction_b, "direction_b", &name)?;
                atoms.push(Atom::DirDot(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: db },
                    0.0,
                ));
            }
            MateKind::TangentSpherePlane {
                center_a,
                radius,
                plane_b,
            } => {
                let ca = finite_point(*center_a, "center_a", &name)?;
                let po = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let pn = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                let r = finite_scalar(*radius, "radius", &name)?;
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: ba, p: ca },
                    LocalPoint { body: bb, p: po },
                    LocalDir { body: bb, d: pn },
                    r.abs(),
                ));
            }
            MateKind::Symmetric { element_a, element_b, plane_c } => {
                let bc = mate
                    .body_c
                    .ok_or_else(|| format!("{name}: a symmetric mate needs body_c (the mirror's body)"))?;
                let o = LocalPoint { body: bc, p: finite_point(plane_c.origin, "plane_c.origin", &name)? };
                let n = LocalDir { body: bc, d: unit_dir(plane_c.normal, "plane_c.normal", &name)? };
                match (element_a, element_b) {
                    (SymmetricElement::Point { point: pa }, SymmetricElement::Point { point: pb }) => {
                        let pa = LocalPoint { body: ba, p: finite_point(*pa, "element_a.point", &name)? };
                        let pb = LocalPoint { body: bb, p: finite_point(*pb, "element_b.point", &name)? };
                        atoms.push(Atom::SymPoints(pa, pb, o, n));
                    }
                    (SymmetricElement::Plane { plane: pa }, SymmetricElement::Plane { plane: pb }) => {
                        let oa = LocalPoint { body: ba, p: finite_point(pa.origin, "element_a.origin", &name)? };
                        let na = LocalDir { body: ba, d: unit_dir(pa.normal, "element_a.normal", &name)? };
                        let ob = LocalPoint { body: bb, p: finite_point(pb.origin, "element_b.origin", &name)? };
                        let nb = LocalDir { body: bb, d: unit_dir(pb.normal, "element_b.normal", &name)? };
                        atoms.push(Atom::SymDirMatch(na, nb, n));
                        atoms.push(Atom::SymPointOnPlane(ob, oa, na, o, n));
                    }
                    (SymmetricElement::Axis { axis: xa }, SymmetricElement::Axis { axis: xb }) => {
                        let oa = LocalPoint { body: ba, p: finite_point(xa.origin, "element_a.origin", &name)? };
                        let da = LocalDir { body: ba, d: unit_dir(xa.direction, "element_a.direction", &name)? };
                        let ob = LocalPoint { body: bb, p: finite_point(xb.origin, "element_b.origin", &name)? };
                        let db = LocalDir { body: bb, d: unit_dir(xb.direction, "element_b.direction", &name)? };
                        atoms.push(Atom::SymDirCross(da, db, n));
                        atoms.push(Atom::SymAxisLateral(oa, ob, db, o, n));
                    }
                    _ => {
                        return Err(format!(
                            "{name}: a symmetric mate pairs like elements (two points, two planes or two axes)"
                        ))
                    }
                }
            }
            MateKind::Gear { axis_a, ref_a, world_ref_a, axis_b, ref_b, world_ref_b, ratio, reverse } => {
                let ratio = finite_scalar(*ratio, "ratio", &name)?;
                let a = spin(ba, axis_a, *ref_a, *world_ref_a, "a", &name)?;
                let b = spin(bb, axis_b, *ref_b, *world_ref_b, "b", &name)?;
                atoms.push(Atom::GearPhase(a, b, ratio, if *reverse { -1.0 } else { 1.0 }));
            }
            MateKind::RackPinion {
                pinion_axis,
                pinion_ref,
                world_pinion_ref,
                rack_point,
                rack_direction,
                world_rack_point,
                distance_per_revolution,
                reverse,
            } => {
                let pinion = spin(ba, pinion_axis, *pinion_ref, *world_pinion_ref, "pinion", &name)?;
                let p = LocalPoint { body: bb, p: finite_point(*rack_point, "rack_point", &name)? };
                let d = LocalDir { body: bb, d: unit_dir(*rack_direction, "rack_direction", &name)? };
                let origin = finite_point(*world_rack_point, "world_rack_point", &name)?;
                let per_turn = finite_scalar(*distance_per_revolution, "distance_per_revolution", &name)?;
                let k = per_turn / std::f64::consts::TAU * if *reverse { -1.0 } else { 1.0 };
                atoms.push(Atom::RackTravel(pinion, p, d, origin, k));
            }
            MateKind::Screw { axis_a, ref_a, ref_b, point_b, pitch, reverse, angle0, offset0 } => {
                let oa = LocalPoint { body: ba, p: finite_point(axis_a.origin, "axis_a.origin", &name)? };
                let da = LocalDir { body: ba, d: unit_dir(axis_a.direction, "axis_a.direction", &name)? };
                let ra = LocalDir { body: ba, d: unit_dir(*ref_a, "ref_a", &name)? };
                let rb = LocalDir { body: bb, d: unit_dir(*ref_b, "ref_b", &name)? };
                let pb = LocalPoint { body: bb, p: finite_point(*point_b, "point_b", &name)? };
                let pitch = finite_scalar(*pitch, "pitch", &name)?;
                let k = pitch / std::f64::consts::TAU * if *reverse { -1.0 } else { 1.0 };
                atoms.push(Atom::ScrewAdvance(
                    oa,
                    da,
                    ra,
                    rb,
                    pb,
                    finite_scalar(*angle0, "angle0", &name)?,
                    finite_scalar(*offset0, "offset0", &name)?,
                    k,
                ));
            }
            MateKind::Limit { mate: inner, min, max } => {
                let min = finite_scalar(*min, "min", &name)?;
                let max = finite_scalar(*max, "max", &name)?;
                atoms.extend(limit_atoms(bodies, ba, bb, inner, min, max, &name)?);
            }
            MateKind::TangentCylinderPlane {
                axis_a,
                radius,
                plane_b,
            } => {
                let oa = finite_point(axis_a.origin, "axis_a.origin", &name)?;
                let da = unit_dir(axis_a.direction, "axis_a.direction", &name)?;
                let po = finite_point(plane_b.origin, "plane_b.origin", &name)?;
                let pn = unit_dir(plane_b.normal, "plane_b.normal", &name)?;
                let r = finite_scalar(*radius, "radius", &name)?;
                // Axis parallel to the plane…
                atoms.push(Atom::DirDot(
                    LocalDir { body: ba, d: da },
                    LocalDir { body: bb, d: pn },
                    0.0,
                ));
                // …at height r above it.
                atoms.push(Atom::PointOnPlane(
                    LocalPoint { body: ba, p: oa },
                    LocalPoint { body: bb, p: po },
                    LocalDir { body: bb, d: pn },
                    r.abs(),
                ));
            }
            MateKind::PointOnCurve { point_a, curve_b, along, reverse } => {
                let pa = finite_point(*point_a, "point_a", &name)?;
                let curve = CurveRef { body: bb, curve: std::sync::Arc::new(SampledCurve::new(curve_b, &name)?) };
                let lp = LocalPoint { body: ba, p: pa };
                atoms.push(Atom::PointOnCurve(lp, curve.clone()));
                if let Some(percent) = along {
                    let percent = finite_scalar(*percent, "along", &name)?;
                    atoms.push(Atom::CurveAlong(lp, None, curve, percent / 100.0, *reverse));
                }
            }
            MateKind::Slot { axis_a, curve_b, normal_b, along, reverse } => {
                let oa = finite_point(axis_a.origin, "axis_a.origin", &name)?;
                let da = unit_dir(axis_a.direction, "axis_a.direction", &name)?;
                let nb = unit_dir(*normal_b, "normal_b", &name)?;
                let curve = CurveRef { body: bb, curve: std::sync::Arc::new(SampledCurve::new(curve_b, &name)?) };
                let (lp, ld) = (LocalPoint { body: ba, p: oa }, LocalDir { body: ba, d: da });
                atoms.push(Atom::DirCross(ld, LocalDir { body: bb, d: nb }));
                atoms.push(Atom::AxisOnCurve(lp, ld, curve.clone()));
                if let Some(percent) = along {
                    let percent = finite_scalar(*percent, "along", &name)?;
                    atoms.push(Atom::CurveAlong(lp, Some(ld), curve, percent / 100.0, *reverse));
                }
            }
            MateKind::Cam { center_a, axis_a, radius, profile_b, normal_b, flip } => {
                let ca = finite_point(*center_a, "center_a", &name)?;
                let nb = unit_dir(*normal_b, "normal_b", &name)?;
                let r = finite_scalar(*radius, "radius", &name)?.abs();
                let sampled = SampledCurve::new(profile_b, &name)?;
                let start = sampled.start();
                // Outside a closed profile is to the right of its direction
                // when it runs counter-clockwise about the normal.
                let side = sampled.orientation(nb) * if *flip { -1.0 } else { 1.0 };
                let curve = CurveRef { body: bb, curve: std::sync::Arc::new(sampled) };
                let (lp, ln) = (LocalPoint { body: ba, p: ca }, LocalDir { body: bb, d: nb });
                let axis = match axis_a {
                    Some(direction) => {
                        let da = unit_dir(*direction, "axis_a", &name)?;
                        let ld = LocalDir { body: ba, d: da };
                        atoms.push(Atom::DirCross(ld, ln));
                        Some(ld)
                    }
                    None => {
                        atoms.push(Atom::PointOnPlane(lp, LocalPoint { body: bb, p: start }, ln, 0.0));
                        None
                    }
                };
                atoms.push(Atom::CamContact(lp, axis, curve, ln, r, side));
            }
        }
        for _ in before..atoms.len() {
            atom_mate.push(mi);
        }
    }
    Ok((atoms, atom_mate))
}

/// Expand a limit mate: `inner`'s rows with its measure (always the LAST atom
/// a measuring kind emits) replaced by the limit row. Angles convert to the
/// measure's own units — the cosine for `Angle` (which reverses the ends),
/// radians for `DirectedAngle`.
fn limit_atoms(
    bodies: &[AssemblyBody],
    ba: usize,
    bb: usize,
    inner: &MateKind,
    min: f64,
    max: f64,
    name: &str,
) -> Result<Vec<Atom>, String> {
    let (lo, hi) = match inner {
        MateKind::Angle { .. } => {
            if min > max {
                return Err(format!("{name}: limit min must not exceed max"));
            }
            (max.to_radians().cos(), min.to_radians().cos())
        }
        MateKind::DirectedAngle { .. } => (min.to_radians(), max.to_radians()),
        MateKind::DistancePointPoint { .. }
        | MateKind::DistancePointPlane { .. }
        | MateKind::DistancePlanePlane { .. }
        | MateKind::DistancePointLine { .. }
        | MateKind::DistanceLineLine { .. } => {
            if min > max {
                return Err(format!("{name}: limit min must not exceed max"));
            }
            (min, max)
        }
        _ => return Err(format!("{name}: only a distance or angle mate can be limited")),
    };
    let single = AssemblyMate { id: Some(name.to_string()), body_a: ba, body_b: bb, body_c: None, kind: inner.clone() };
    let (mut atoms, _) = expand_mates(bodies, std::slice::from_ref(&single))?;
    let measure = atoms
        .pop()
        .as_ref()
        .and_then(Measure::of)
        .ok_or_else(|| format!("{name}: the limited mate measures nothing"))?;
    atoms.push(Atom::Limit(measure, lo, hi));
    Ok(atoms)
}

/// A spin atom side: `axis` and the local reference ⟂ it on `body`, read
/// against the world reference `world`.
fn spin(body: usize, axis: &MateAxis, reference: [f64; 3], world: [f64; 3], side: &str, name: &str) -> Result<Spin, String> {
    let n = unit_dir(axis.direction, &format!("{side} axis direction"), name)?;
    let r = finite_point(reference, &format!("{side} reference"), name)?;
    let r = r.sub(n.scale(r.dot(n)));
    if !(r.length() > ASM_DIR_EPS) {
        return Err(format!("{name}: the {side} reference must not lie along its axis"));
    }
    let world = finite_point(world, &format!("{side} world reference"), name)?;
    Ok(Spin { reference: LocalDir { body, d: r.scale(1.0 / r.length()) }, axis: LocalDir { body, d: n }, world })
}

impl MateKind {
    /// Re-capture a mechanical mate's references so it holds exactly the
    /// poses given (`(rotation [w,x,y,z], translation)` of body A and body B):
    /// the gear's and the pinion's world reference directions, the rack's
    /// world point, the screw's angle and offset. Every other kind is left
    /// alone. Mapping calls this at the current poses, so a new mate moves
    /// nothing; a live drag calls it at the last good poses, so each frame
    /// couples only the motion since then and never wraps a spin.
    /// A mechanical mate's captured references as plain numbers (the world
    /// reference directions / point, or the screw's angle and offset), for
    /// persisting across runs; `None` for every other kind.
    pub fn references(&self) -> Option<Vec<f64>> {
        match self {
            MateKind::Gear { world_ref_a, world_ref_b, .. } => Some([&world_ref_a[..], &world_ref_b[..]].concat()),
            MateKind::RackPinion { world_pinion_ref, world_rack_point, .. } => {
                Some([&world_pinion_ref[..], &world_rack_point[..]].concat())
            }
            // The nut's reference direction travels with the angle it was
            // captured against: the mapping derives `ref_b` from the poses at
            // map time, so a later run would pair a fresh `ref_b` with an old
            // `angle0` and read the screw as turned.
            MateKind::Screw { ref_b, angle0, offset0, .. } => Some(vec![*angle0, *offset0, ref_b[0], ref_b[1], ref_b[2]]),
            _ => None,
        }
    }

    /// Restore references saved by [`MateKind::references`]; `false` (and no
    /// change) when the kind or the count does not match.
    pub fn set_references(&mut self, values: &[f64]) -> bool {
        let triple = |v: &[f64]| [v[0], v[1], v[2]];
        match self {
            MateKind::Gear { world_ref_a, world_ref_b, .. } if values.len() == 6 => {
                *world_ref_a = triple(&values[0..3]);
                *world_ref_b = triple(&values[3..6]);
                true
            }
            MateKind::RackPinion { world_pinion_ref, world_rack_point, .. } if values.len() == 6 => {
                *world_pinion_ref = triple(&values[0..3]);
                *world_rack_point = triple(&values[3..6]);
                true
            }
            // Two values: saved before `ref_b` was persisted; the mapped
            // `ref_b` stands.
            MateKind::Screw { ref_b, angle0, offset0, .. } if values.len() == 2 || values.len() == 5 => {
                *angle0 = values[0];
                *offset0 = values[1];
                if values.len() == 5 {
                    *ref_b = triple(&values[2..5]);
                }
                true
            }
            _ => false,
        }
    }

    /// Which side of a gear or rack-and-pinion moved since its references
    /// were captured, when exactly one did: `Some(true)` for body A, `Some(false)`
    /// for body B. A typed edit of one gear makes that gear the driver.
    pub fn driver(&self, pose_a: ([f64; 4], [f64; 3]), pose_b: ([f64; 4], [f64; 3])) -> Option<bool> {
        let rotate = |pose: ([f64; 4], [f64; 3]), v: [f64; 3]| quat_rotate(pose.0, vec3(v));
        let place = |pose: ([f64; 4], [f64; 3]), p: [f64; 3]| rotate(pose, p).add(vec3(pose.1));
        let unit = |v: Vec3| v.scale(1.0 / v.length().max(ASM_DIR_EPS));
        let (moved_a, moved_b) = match self {
            MateKind::Gear { axis_a, ref_a, world_ref_a, axis_b, ref_b, world_ref_b, .. } => (
                signed_angle(vec3(*world_ref_a), rotate(pose_a, *ref_a), unit(rotate(pose_a, axis_a.direction))).abs(),
                signed_angle(vec3(*world_ref_b), rotate(pose_b, *ref_b), unit(rotate(pose_b, axis_b.direction))).abs(),
            ),
            MateKind::RackPinion { pinion_axis, pinion_ref, world_pinion_ref, rack_point, world_rack_point, .. } => (
                signed_angle(vec3(*world_pinion_ref), rotate(pose_a, *pinion_ref), unit(rotate(pose_a, pinion_axis.direction))).abs(),
                place(pose_b, *rack_point).sub(vec3(*world_rack_point)).length(),
            ),
            _ => return None,
        };
        const STILL: f64 = 1e-9;
        match (moved_a > STILL, moved_b > STILL) {
            (true, false) => Some(true),
            (false, true) => Some(false),
            _ => None,
        }
    }

    pub fn rebase(&mut self, pose_a: ([f64; 4], [f64; 3]), pose_b: ([f64; 4], [f64; 3])) {
        let rotate = |pose: ([f64; 4], [f64; 3]), v: [f64; 3]| quat_rotate(pose.0, vec3(v));
        let place = |pose: ([f64; 4], [f64; 3]), p: [f64; 3]| rotate(pose, p).add(vec3(pose.1));
        let triple = |v: Vec3| [v.x, v.y, v.z];
        match self {
            MateKind::Gear { ref_a, world_ref_a, ref_b, world_ref_b, .. } => {
                *world_ref_a = triple(rotate(pose_a, *ref_a));
                *world_ref_b = triple(rotate(pose_b, *ref_b));
            }
            MateKind::RackPinion { pinion_ref, world_pinion_ref, rack_point, world_rack_point, .. } => {
                *world_pinion_ref = triple(rotate(pose_a, *pinion_ref));
                *world_rack_point = triple(place(pose_b, *rack_point));
            }
            MateKind::Screw { axis_a, ref_a, ref_b, point_b, angle0, offset0, .. } => {
                let n = rotate(pose_a, axis_a.direction);
                let n = n.scale(1.0 / n.length().max(ASM_DIR_EPS));
                *angle0 = signed_angle(rotate(pose_a, *ref_a), rotate(pose_b, *ref_b), n);
                *offset0 = place(pose_b, *point_b).sub(place(pose_a, axis_a.origin)).dot(n);
            }
            _ => {}
        }
    }
}
