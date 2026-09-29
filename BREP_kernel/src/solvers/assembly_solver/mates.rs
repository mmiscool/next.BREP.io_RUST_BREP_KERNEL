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
        }
        for _ in before..atoms.len() {
            atom_mate.push(mi);
        }
    }
    Ok((atoms, atom_mate))
}
