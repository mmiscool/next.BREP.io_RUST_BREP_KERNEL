use super::*;
use crate::{classify_point, PointClass, SolidClassifier};
use serde::{Deserialize, Serialize};

/// Rational ruled surface between two curves with IDENTICAL degree, knots and
/// weight pattern (the drafted-arc rows share one `make_arc` window, so this
/// yields the EXACT cone patch: each ruling blends radially-corresponding
/// points at equal weights).
pub(super) fn ruled_between(bottom: &NurbsCurve, top: &NurbsCurve) -> Result<NurbsSurface, KernelRefusal> {
    if bottom.degree != top.degree
        || bottom.control_points.len() != top.control_points.len()
        || bottom.knots.len() != top.knots.len()
    {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "ruled_rows",
            "ruled_between: rows are not representation-compatible",
        ));
    }
    let grid = bottom
        .control_points
        .iter()
        .zip(&top.control_points)
        .map(|(a, b)| vec![*a, *b])
        .collect();
    NurbsSurface::new(
        bottom.degree,
        1,
        bottom.knots.clone(),
        vec![0.0, 0.0, 1.0, 1.0],
        grid,
    )
    .or_refuse(KernelStage::Fragment, "surface_new")
}

/// Subrange of a wall ROW curve between the projections of two junction
/// points.  Splitting (instead of rebuilding with `make_arc`) preserves the
/// row's parameterization exactly, so the edge is the surface's own boundary
/// restriction and its parameter-line pcurve is pointwise exact.
pub(super) fn arc_window_subrange(row: &NurbsCurve, start: Vec3, end: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = row.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let span = d1 - d0;
    let u0 = project_point_to_curve(row, start).or_refuse(KernelStage::Fragment, "project_point")?.u;
    let u1 = project_point_to_curve(row, end).or_refuse(KernelStage::Fragment, "project_point")?.u;
    if u1 <= u0 + 1e-12 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "arc_trim",
            "draftExtrude: a drafted arc's boundary trim inverted",
        ));
    }
    let epsilon = span * 1e-9;
    let mut current = row.clone();
    if u0 > d0 + epsilon {
        current = current.split(u0).or_refuse(KernelStage::Fragment, "split")?.1;
    }
    let domain = current.domain().or_refuse(KernelStage::Fragment, "domain")?;
    if u1 < domain[1] - epsilon && u1 > domain[0] + epsilon {
        current = current.split(u1).or_refuse(KernelStage::Fragment, "split")?.0;
    }
    Ok(current)
}

/// Gradient (unnormalized surface normal direction) of a drafted WALL's
/// implicit surface at a point on it: for a line wall the tilted plane's
/// normal; for an arc wall the cone's ∇(ρ − r(z)) = ρ̂ + (d·turn/h)·ẑ.  Exact
/// closed forms — the junction-conic end tangents come from their cross
/// products.
fn wall_gradient(
    seg: &SegGeom,
    point: Vec3,
    zh: Vec3,
    height: f64,
    signed_d: f64,
) -> Result<Vec3, KernelRefusal> {
    match seg {
        SegGeom::Line { dir, normal, .. } => dir
            .cross(normal.scale(signed_d).add(zh.scale(height)))
            .normalized()
            .or_refuse(KernelStage::Fragment, "normalized"),
        SegGeom::Arc {
            center, turn, ..
        } => {
            let rel = point.sub(*center);
            let radial = rel.sub(zh.scale(rel.dot(zh)));
            let rho = radial.normalized().or_refuse(KernelStage::Fragment, "normalized")?;
            rho.add(zh.scale(signed_d * turn / height))
                .normalized()
                .or_refuse(KernelStage::Fragment, "normalized")
        }
    }
}

/// The EXACT junction edge between two adjacent drafted walls, from bottom
/// junction `a` to top junction `b` with mid-height witness `m` (all three are
/// exact offset-primitive intersections).  Straight when `m` is collinear
/// (plane∧plane miter edges, tangent-junction cone rulings); otherwise the
/// exact CONIC through `a`/`b` with the walls' analytic gradient-cross end
/// tangents, its rational-quadratic weight solved from `m` (a conic is
/// uniquely determined by that data, and every drafted wall∧wall intersection
/// IS a conic — both squared implicits shrink linearly at the same rate, so
/// their difference is a plane).
#[allow(clippy::too_many_arguments)]
pub(super) fn junction_edge_curve(
    prev: &SegGeom,
    next: &SegGeom,
    a: Vec3,
    m: Vec3,
    b: Vec3,
    zh: Vec3,
    height: f64,
    signed_d: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let chord = b.sub(a);
    let length = chord.length();
    if length <= 1e-12 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "junction_collapsed",
            "draftExtrude: a junction edge collapsed to a point",
        ));
    }
    let along = m.sub(a).dot(chord) / (length * length);
    let deviation = m.sub(a).sub(chord.scale(along)).length();
    if deviation <= length * 1e-9 {
        return make_line(a, b).or_refuse(KernelStage::Fragment, "make_line");
    }
    // 2D frame in the conic's plane (it contains a, b, m by construction).
    let e1 = chord.scale(1.0 / length);
    let plane_normal = chord
        .cross(m.sub(a))
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let e2 = plane_normal
        .cross(e1)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let orient = |tangent: Vec3| {
        if tangent.dot(zh) < 0.0 {
            tangent.scale(-1.0)
        } else {
            tangent
        }
    };
    let t0 = orient(wall_gradient(prev, a, zh, height, signed_d)?
        .cross(wall_gradient(next, a, zh, height, signed_d)?));
    let t2 = orient(wall_gradient(prev, b, zh, height, signed_d)?
        .cross(wall_gradient(next, b, zh, height, signed_d)?));
    let d0 = (t0.dot(e1), t0.dot(e2));
    let d2 = (t2.dot(e1), t2.dot(e2));
    let denom = d0.0 * d2.1 - d0.1 * d2.0;
    let scale0 = d0.0.hypot(d0.1);
    let scale2 = d2.0.hypot(d2.1);
    if denom.abs() <= 1e-14 * scale0 * scale2 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "conic_apex",
            "draftExtrude: junction end tangents are parallel — no conic apex",
        ));
    }
    // Apex: a + s·t0 = b + r·t2 solved in 2D (a = origin, b = (length, 0)).
    let s = length * d2.1 / denom;
    let apex = (s * d0.0, s * d0.1);
    if apex.1.abs() <= f64::EPSILON * length {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "conic_apex_degenerate",
            "draftExtrude: junction conic apex is degenerate",
        ));
    }
    // Barycentric coordinates of m over (a, apex, b): m = α·a + β·apex + γ·b.
    let mq = (m.sub(a).dot(e1), m.sub(a).dot(e2));
    let beta = mq.1 / apex.1;
    let gamma = (mq.0 - beta * apex.0) / length;
    let alpha = 1.0 - beta - gamma;
    if !(alpha > 0.0 && beta > 0.0 && gamma > 0.0) {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "conic_witness",
            format!(
            "draftExtrude: junction conic witness fell outside its control triangle \
             (α={alpha:.3e} β={beta:.3e} γ={gamma:.3e})"
            ),
        ));
    }
    let weight = beta / (2.0 * (alpha * gamma).sqrt());
    let apex_3d = a.add(e1.scale(apex.0)).add(e2.scale(apex.1));
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Vec4::from_point(a, 1.0),
            Vec4::from_point(apex_3d, weight),
            Vec4::from_point(b, 1.0),
        ],
    )
    .or_refuse(KernelStage::Fragment, "curve_new")
}

/// In-plane circumcircle of three coplanar points (projected onto `ex`/`ey`,
/// `np = ex×ey`).  Returns `None` when the three points are collinear.
fn circumcircle(a: Vec3, b: Vec3, c: Vec3, ex: Vec3, ey: Vec3, np: Vec3) -> Option<(Vec3, f64)> {
    let (ax, ay) = (a.dot(ex), a.dot(ey));
    let (bx, by) = (b.dot(ex), b.dot(ey));
    let (cx, cy) = (c.dot(ex), c.dot(ey));
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return None;
    }
    let a2 = ax * ax + ay * ay;
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let plane_off = a.dot(np);
    let center = ex.scale(ux).add(ey.scale(uy)).add(np.scale(plane_off));
    let radius = a.sub(center).length();
    Some((center, radius))
}

/// Intersect an offset LINE (through `line_point`, direction `line_dir`) with an
/// offset CIRCLE (`center`, `radius`), returning the root nearest `near` (the
/// original junction).  A clear Err when they no longer meet (offset too large).
fn intersect_offset_line_circle(
    line_point: Vec3,
    line_dir: Vec3,
    center: Vec3,
    radius: f64,
    near: Vec3,
) -> Result<Vec3, KernelRefusal> {
    let dir = line_dir.normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let f = line_point.sub(center);
    let b = f.dot(dir);
    let c = f.dot(f) - radius * radius;
    let disc = b * b - c;
    if disc < -1e-9 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "offset_too_large",
            "an offset line and arc no longer meet (offset too large)",
        ));
    }
    let root = disc.max(0.0).sqrt();
    let p1 = line_point.add(dir.scale(-b + root));
    let p2 = line_point.add(dir.scale(-b - root));
    Ok(if p1.sub(near).length() <= p2.sub(near).length() {
        p1
    } else {
        p2
    })
}

/// Intersect two offset CIRCLES (in the plane whose normal is `plane_normal`),
/// returning the root nearest `near`.  A clear Err when they are concentric or no
/// longer meet.
fn intersect_offset_circles(
    c1: Vec3,
    r1: f64,
    c2: Vec3,
    r2: f64,
    plane_normal: Vec3,
    near: Vec3,
) -> Result<Vec3, KernelRefusal> {
    let between = c2.sub(c1);
    let d = between.length();
    if d < 1e-9 {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "concentric_arcs",
            "concentric offset arcs do not meet",
        ));
    }
    let axis = between.scale(1.0 / d);
    let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let h2 = r1 * r1 - a * a;
    if h2 < -1e-9 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "offset_arcs_too_large",
            "offset arcs no longer meet (offset too large)",
        ));
    }
    let h = h2.max(0.0).sqrt();
    let base = c1.add(axis.scale(a));
    let perp = plane_normal.cross(axis).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let p1 = base.add(perp.scale(h));
    let p2 = base.sub(perp.scale(h));
    Ok(if p1.sub(near).length() <= p2.sub(near).length() {
        p1
    } else {
        p2
    })
}

/// Geometry class of one profile segment, shared by the draft-extrude builder
/// and the in-plane offset engine: a straight LINE or a circular ARC in the
/// plane with normal `plane_normal`.
pub(super) enum SegGeom {
    Line {
        start: Vec3,
        end: Vec3,
        /// Unit chord direction.
        dir: Vec3,
        /// In-plane offset normal `plane_normal × dir` (inward on a CCW loop).
        normal: Vec3,
    },
    Arc {
        center: Vec3,
        radius: f64,
        /// +1 when the arc bends CCW about the plane normal (a convex arc on a
        /// CCW loop, which SHRINKS under a positive inward offset), −1 when CW.
        turn: f64,
        /// ±plane_normal — the axis the arc sweeps CCW about.
        arc_normal: Vec3,
        start: Vec3,
        end: Vec3,
    },
}

impl SegGeom {
    fn start(&self) -> Vec3 {
        match self {
            SegGeom::Line { start, .. } | SegGeom::Arc { start, .. } => *start,
        }
    }

    fn end(&self) -> Vec3 {
        match self {
            SegGeom::Line { end, .. } | SegGeom::Arc { end, .. } => *end,
        }
    }

    /// Naive offset image of a point ON this segment's primitive: lines
    /// translate along their normal; arc points scale radially onto the
    /// concentric offset circle (Err when a concave arc collapses).
    fn offset_point(&self, point: Vec3, signed_d: f64) -> Result<Vec3, KernelRefusal> {
        match self {
            SegGeom::Line { normal, .. } => Ok(point.add(normal.scale(signed_d))),
            SegGeom::Arc {
                center,
                radius,
                turn,
                ..
            } => {
                let r_offset = radius - signed_d * turn;
                if r_offset <= 1e-6 {
                    return Err(KernelRefusal::input(
                        KernelStage::Classify,
                        "arc_collapse",
                        "offset: distance is too large — a concave arc collapses",
                    ));
                }
                Ok(center.add(point.sub(*center).scale(r_offset / radius)))
            }
        }
    }
}

/// Classify every profile segment as a LINE (all interior samples on the
/// chord) or a circular ARC (circumcircle through start/mid/end confirmed by
/// on-circle samples), with its turning direction about `plane_normal`.
/// Anything else is a clear Err — the offset/draft machinery is exact for
/// lines and circles only.
pub(super) fn classify_profile_segments(
    profile: &[NurbsCurve],
    plane_normal: Vec3,
) -> Result<Vec<SegGeom>, KernelRefusal> {
    let tol = 1e-6;
    if profile.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_empty",
            "offset: profile has no segments",
        ));
    }
    let np = plane_normal.normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let ex = np.perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?;
    let ey = np.cross(ex).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let mut segs = Vec::with_capacity(profile.len());
    for curve in profile {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        let start = curve.evaluate(t0).or_refuse(KernelStage::Classify, "evaluate")?;
        let end = curve.evaluate(t1).or_refuse(KernelStage::Classify, "evaluate")?;
        let chord = end.sub(start);
        let chord_len = chord.length();
        if chord_len <= tol {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "segment_length",
                "offset: profile has a degenerate (zero-length) segment",
            ));
        }
        let dir = chord.scale(1.0 / chord_len);
        // A straight LINE if every interior sample lies on the chord.
        let mut is_line = true;
        for k in 1..8 {
            let point = curve.evaluate(t0 + (t1 - t0) * k as f64 / 8.0).or_refuse(KernelStage::Classify, "evaluate")?;
            let rel = point.sub(start);
            let perpendicular = rel.sub(dir.scale(rel.dot(dir))).length();
            if perpendicular > tol * 10.0 {
                is_line = false;
                break;
            }
        }
        if is_line {
            segs.push(SegGeom::Line {
                start,
                end,
                dir,
                normal: np.cross(dir).normalized().or_refuse(KernelStage::Classify, "normalized")?,
            });
            continue;
        }
        // Otherwise it must be a circular ARC: fit a circle through start/mid/end.
        let mid = curve.evaluate((t0 + t1) * 0.5).or_refuse(KernelStage::Classify, "evaluate")?;
        let (center, radius) = circumcircle(start, mid, end, ex, ey, np).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "segment_kind",
                "offset: only straight lines and circular arcs are supported",
            )
        })?;
        for k in 0..=8 {
            let point = curve.evaluate(t0 + (t1 - t0) * k as f64 / 8.0).or_refuse(KernelStage::Classify, "evaluate")?;
            if (point.sub(center).length() - radius).abs() > tol * 10.0 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "segment_kind",
                    "offset: only straight lines and circular arcs are supported",
                ));
            }
        }
        // Turning direction about the plane normal (CCW ⇒ +1 ⇒ shrink inward).
        let bend = np.dot(mid.sub(start).cross(end.sub(mid)));
        let (arc_normal, turn) = if bend >= 0.0 {
            (np, 1.0)
        } else {
            (np.scale(-1.0), -1.0)
        };
        segs.push(SegGeom::Arc {
            center,
            radius,
            turn,
            arc_normal,
            start,
            end,
        });
    }
    Ok(segs)
}

/// The junction between two consecutive segments' OFFSET primitives at signed
/// in-plane distance `signed_d`:
///   • offset 0 → the original shared vertex, exactly;
///   • a TANGENT (G1) junction degenerates to the shared offset point
///     (coincident-naive-offsets fast path);
///   • line∧line → the exact miter V' = V + signed_d/(1+nₐ·n_b)·(nₐ+n_b);
///   • line∧arc  → the offset line ∩ the offset circle, root nearest V;
///   • arc∧arc   → the two offset circles ∩, root nearest V.
/// Offsets that no longer meet (too large for the local feature) are a clear
/// Err.
pub(super) fn offset_junction(
    prev: &SegGeom,
    next: &SegGeom,
    plane_normal: Vec3,
    signed_d: f64,
) -> Result<Vec3, KernelRefusal> {
    let vertex = prev.end();
    if signed_d == 0.0 {
        return Ok(vertex);
    }
    let prev_offset = prev.offset_point(prev.end(), signed_d)?;
    let next_offset = next.offset_point(next.start(), signed_d)?;
    if prev_offset.sub(next_offset).length() <= 1e-6 {
        return Ok(prev_offset.add(next_offset).scale(0.5));
    }
    match (prev, next) {
        (SegGeom::Line { normal: na, .. }, SegGeom::Line { normal: nb, .. }) => {
            let denom = 1.0 + na.dot(*nb);
            if denom.abs() < 1e-6 {
                return Err(KernelRefusal::input(
                    KernelStage::Classify,
                    "reversal_corner",
                    "offset: degenerate (near-reversal) polyline corner",
                ));
            }
            Ok(vertex.add(na.add(*nb).scale(signed_d / denom)))
        }
        (
            SegGeom::Line { dir, .. },
            SegGeom::Arc {
                center,
                radius,
                turn,
                ..
            },
        ) => intersect_offset_line_circle(
            prev_offset,
            *dir,
            *center,
            radius - signed_d * turn,
            vertex,
        ),
        (
            SegGeom::Arc {
                center,
                radius,
                turn,
                ..
            },
            SegGeom::Line { dir, .. },
        ) => intersect_offset_line_circle(
            next_offset,
            *dir,
            *center,
            radius - signed_d * turn,
            vertex,
        ),
        (
            SegGeom::Arc {
                center: c1,
                radius: r1,
                turn: turn1,
                ..
            },
            SegGeom::Arc {
                center: c2,
                radius: r2,
                turn: turn2,
                ..
            },
        ) => intersect_offset_circles(
            *c1,
            r1 - signed_d * turn1,
            *c2,
            r2 - signed_d * turn2,
            plane_normal,
            vertex,
        ),
    }
}

/// Offset a planar profile CHAIN (a sequence of LINE and circular-ARC segments)
/// IN-PLANE by the SIGNED distance `signed_d` along the per-segment offset normal
/// n = (plane_normal × tangent).normalized().  Lines move to a parallel line;
/// circular arcs move to a CONCENTRIC arc (radius r' = r − signed_d·turn — a
/// convex-outward arc shrinks, a convex-inward arc grows).  Consecutive offset
/// segments are re-joined at their [`offset_junction`].  `closed` treats the
/// chain as a loop (every junction re-joined); an OPEN chain leaves its two end
/// offsets un-joined.  Returns one reconstructed NurbsCurve per input segment
/// (make_line / make_arc).  A self-intersecting offset (signed_d too large for
/// a concave corner or arc) surfaces as a clear Err.
fn offset_profile_segments(
    profile: &[NurbsCurve],
    plane_normal: Vec3,
    signed_d: f64,
    closed: bool,
) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    let tol = 1e-6;
    let np = plane_normal.normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let segs = classify_profile_segments(profile, np)?;
    let n = segs.len();

    // Naive per-segment offsets, then re-join consecutive ones exactly.
    let mut offsets: Vec<(Vec3, Vec3)> = segs
        .iter()
        .map(|seg| {
            Ok((
                seg.offset_point(seg.start(), signed_d)?,
                seg.offset_point(seg.end(), signed_d)?,
            ))
        })
        .collect::<Result<_, KernelRefusal>>()?;
    let junctions = if closed { n } else { n.saturating_sub(1) };
    for i in 0..junctions {
        let j = (i + 1) % n;
        let point = offset_junction(&segs[i], &segs[j], np, signed_d)?;
        offsets[i].1 = point;
        offsets[j].0 = point;
    }

    // --- Reconstruct each offset segment as a NurbsCurve.
    let mut out = Vec::with_capacity(n);
    for (seg, (off_start, off_end)) in segs.iter().zip(&offsets) {
        match seg {
            SegGeom::Line { .. } => out.push(make_line(*off_start, *off_end).or_refuse(KernelStage::Fragment, "make_line")?),
            SegGeom::Arc {
                center, arc_normal, ..
            } => {
                let radial = off_start.sub(*center);
                let r2 = radial.length();
                if r2 <= tol {
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "arc_radius",
                        "offset: reconstructed arc has a zero radius",
                    ));
                }
                let ax = radial.scale(1.0 / r2);
                let ay = arc_normal.cross(ax).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
                let ve = off_end.sub(*center);
                let mut angle = ve.dot(ay).atan2(ve.dot(ax));
                if angle <= 1e-9 {
                    angle += std::f64::consts::TAU;
                }
                out.push(make_arc(*center, ax, ay, r2, 0.0, angle).or_refuse(KernelStage::Fragment, "make_arc")?);
            }
        }
    }
    Ok(out)
}

/// Which way a rib grows out of its sketch — SolidWorks' **Extrusion Direction**
/// control, and the same two choices it offers.
///
/// The two are not variants of one construction: they SWAP which axis carries the
/// thickness and which carries the growth, which is why a rib built in the wrong
/// one lies down where it should stand up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RibExtrusion {
    /// **Parallel to Sketch** (SolidWorks' default, and what everyone means by a
    /// rib or a gusset): the material grows PARALLEL to the sketch plane and the
    /// thickness is applied NORMAL to it. A line drawn between two walls becomes a
    /// thin fin standing ON the sketch plane, growing across it until it lands on
    /// the part.
    #[default]
    ParallelToSketch,
    /// **Normal to Sketch**: the material grows NORMAL to the sketch plane and the
    /// thickness is applied IN it. The chain is thickened inside its own plane and
    /// that ribbon is driven off the plane — walls hanging under a sketch.
    NormalToSketch,
}

/// Rib / stiffener (§6.6) — SolidWorks' Rib, both extrusion directions, with its
/// **Up To Next** end condition.
///
/// The chain is thickened by `thickness` and grown along `extrude_dir` until it
/// LANDS ON THE PART. Which axis carries which is [`RibExtrusion`]'s whole
/// purpose: `ParallelToSketch` offsets ±thickness/2 along the plane NORMAL and
/// sweeps the chain IN the plane; `NormalToSketch` offsets ±thickness/2 INSIDE
/// the plane (miter-joined, straight caps across the open ends) and sweeps that
/// ribbon along the plane normal.
///
/// # Up To Next
///
/// There is no depth. SolidWorks' rib has exactly one end condition — the rib
/// develops until it meets the next faces and the feature FAILS if any part of it
/// meets nothing — and this reproduces it exactly rather than approximating it:
/// the chain is swept a generous `reach` (twice the part's bounding diagonal, so
/// it certainly crosses the part), the sweep is CUT BY THE PART
/// (`slab − solid`), and the piece that grew out of the sketch is kept. Its
/// termination surface is therefore the part's own faces, whatever shape they are.
/// A piece still running at `reach` never landed, and that is the documented
/// failure — not a silently truncated rib.
///
/// `plane_normal` is the profile's own plane when the caller knows it (a sketch
/// publishes the plane it was drawn on). `None` falls back to deriving the plane
/// from the chain's bends, which no single straight segment can supply.
///
/// # Face names
///
/// Every face of the result is named, and every name is a pure function of
/// `names` and the geometry — never of the order the booleans met the faces in.
/// See [`RibNames`] for the table.
///
/// V1 SCOPE: POLYLINE profiles only — arcs/curves return a clear Err.
pub fn rib_from_profile(
    solid: &BrepSolid,
    profile: &[NurbsCurve],
    thickness: f64,
    extrude_dir: Vec3,
    plane_normal: Option<Vec3>,
    extrusion: RibExtrusion,
    names: &RibNames,
) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = 1e-6;
    if profile.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_count",
            "rib: profile needs at least 1 curve forming an open chain",
        ));
    }
    if names.segments.len() != profile.len() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "segment_names",
            format!(
                "rib: {} segment names for {} profile curves",
                names.segments.len(),
                profile.len()
            ),
        ));
    }
    if !(thickness > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "thickness",
            "rib: thickness must be positive",
        ));
    }

    // --- 1. Extract the ordered chain vertices (segment endpoints) and verify the
    //        chain is connected end→start.  Segments may be LINES or circular ARCS.
    let mut vertices = Vec::with_capacity(profile.len() + 1);
    let mut samples = Vec::new();
    for (index, curve) in profile.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        let v_start = curve.evaluate(start).or_refuse(KernelStage::Collect, "evaluate")?;
        let v_end = curve.evaluate(end).or_refuse(KernelStage::Collect, "evaluate")?;
        if v_end.sub(v_start).length() <= tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "segment_length",
                "rib: profile has a degenerate (zero-length) segment",
            ));
        }
        if index == 0 {
            vertices.push(v_start);
        } else if v_start.sub(*vertices.last().unwrap()).length() > tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "chain_connected",
                format!("rib: profile chain is not connected at curve {index}"),
            ));
        }
        vertices.push(v_end);
        // Skip k = 0 after the first segment: it duplicates the previous
        // segment's endpoint, which would otherwise inject a zero-length step
        // and cancel the corner bend used to derive the plane normal.
        let first_k = if index == 0 { 0 } else { 1 };
        for k in first_k..=8 {
            samples.push(curve.evaluate(start + (end - start) * k as f64 / 8.0).or_refuse(KernelStage::Collect, "evaluate")?);
        }
    }
    let count = vertices.len();
    if count < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "vertex_count",
            "rib: profile needs at least 2 distinct vertices",
        ));
    }

    // --- 2. A rib thickens an OPEN profile; a closed loop is an ordinary
    //        extrude, not a rib.
    if vertices[count - 1].sub(vertices[0]).length() <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_closed",
            "rib: profile chain is closed; rib expects an open chain",
        ));
    }

    // --- 3. The profile plane normal np.  A caller-supplied plane wins: it is the
    //        plane the profile was AUTHORED on (a sketch publishes it), so it is
    //        both unambiguous in sign and defined for a chain with no bend at all.
    //        Otherwise derive it from the sampled chain's bends (robust for arc
    //        segments), which a fully collinear chain cannot yield.  Either way
    //        the chain must lie in the plane.
    let np = match plane_normal {
        Some(supplied) => supplied.normalized().map_err(|_| {
            KernelRefusal::input(
                KernelStage::Collect,
                "plane_normal",
                "rib: the supplied profile plane normal is degenerate",
            )
        })?,
        None => {
            let mut normal = Vec3::default();
            for i in 1..samples.len() - 1 {
                let a = samples[i].sub(samples[i - 1]);
                let b = samples[i + 1].sub(samples[i]);
                normal = normal.add(a.cross(b));
            }
            // A collinear chain lies in every plane through its line: the
            // input does not pick one, so the rib refuses rather than choosing.
            normal.normalized().map_err(|_| {
                KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "plane_undetermined",
                    "rib: profile is collinear and no profile plane was supplied; cannot determine \
                 its plane",
                )
            })?
        }
    };
    let origin = vertices[0];
    if samples
        .iter()
        .any(|point| point.sub(origin).dot(np).abs() > tolerance * 100.0)
    {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_planar",
            if plane_normal.is_some() {
                "rib: profile does not lie in the supplied plane"
            } else {
                "rib: profile is not planar"
            },
        ));
    }

    // --- 4. How far to sweep before the part cuts the rib back: twice the part's
    //        bounding diagonal certainly crosses it from anywhere on the chain, so
    //        the CUT decides the rib's extent, never this number.
    let reach = sweep_reach(solid, &vertices)?;

    // --- 5. Build the over-long slab for the requested extrusion direction.  The
    //        two arms differ only in which axis carries the thickness.
    let mut slab = match extrusion {
        RibExtrusion::ParallelToSketch => {
            // The growth direction lies IN the plane; anything out of plane is a
            // caller error, not something to silently project away.
            let along = extrude_dir.sub(np.scale(extrude_dir.dot(np)));
            let along = along.normalized().map_err(|_| {
                KernelRefusal::input(
                    KernelStage::Collect,
                    "direction_in_plane",
                    "rib: a Parallel-to-Sketch rib grows INSIDE its sketch plane, but the requested \
                 direction is perpendicular to it",
                )
            })?;
            parallel_slab(profile, &vertices, np, along, thickness, reach)?
        }
        RibExtrusion::NormalToSketch => {
            let along = extrude_dir.normalized().map_err(|_| {
                KernelRefusal::input(
                    KernelStage::Collect,
                    "direction",
                    "rib: extrude direction is degenerate",
                )
            })?;
            normal_slab(profile, np, along, thickness, reach)?
        }
    };

    // --- 6. Up To Next: cut the over-long slab by the part and keep the piece the
    //        sketch grew.  That piece is the rib; its far end IS the part's faces.
    let along = match extrusion {
        RibExtrusion::ParallelToSketch => {
            let along = extrude_dir.sub(np.scale(extrude_dir.dot(np)));
            along.normalized().or_refuse(KernelStage::Collect, "normalized")?
        }
        RibExtrusion::NormalToSketch => extrude_dir.normalized().or_refuse(KernelStage::Collect, "normalized")?,
    };

    // Both operands go through the cut and the fuse carrying SOURCE TOKENS, not
    // names: the booleans number split fragments in the order they meet them,
    // and a part may already hold a `Floor` and a `Floor_1`, so a propagated name
    // cannot say which face a fragment came from. The tokens can, and
    // `restamp_rib_faces` turns them into names once the result exists.
    let roles = slab_face_names(names, extrusion, profile.len());
    let slab_faces: usize = slab.shells.iter().map(|shell| shell.faces.len()).sum();
    if slab_faces != roles.len() {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "slab_faces",
            format!(
                "rib: the slab builder produced {slab_faces} faces, expected {}",
                roles.len()
            ),
        ));
    }
    let (part, part_names) = tokenised(solid, TARGET_TOKEN);
    for (index, face) in slab.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()).enumerate() {
        face.name = Some(format!("{SOURCE_TOKEN}{RIB_TOKEN}{index}"));
    }
    let sources = FaceSources {
        part: part_names,
        rib: roles,
    };

    let seeds = chain_probe_seeds(profile)?;
    let rib = up_to_next(&part, &slab, &seeds, along, reach)
        .map_err(|error| error.with_message(|error| sources.detokenise(error)))?;
    let Some(rib) = rib else {
        // Every bit of the sweep was already material: the rib adds nothing, and
        // the part is its own answer.  Not an error — the same document with a
        // thicker wall would build exactly this.
        return Ok(solid.clone());
    };

    let mut fused = boolean_operation(
        &part,
        &rib,
        BooleanOperation::Union,
        &BooleanOptions::default(),
    )
    .map_err(|error| {
        error.with_message(|error| {
            format!(
                "rib: union of the rib into the part failed: {}",
                sources.detokenise(error)
            )
        })
    })?;
    let chord = vertices[count - 1].sub(vertices[0]).normalized().or_refuse(KernelStage::Collect, "normalized")?;
    let frame = RibFrame {
        origin: vertices[0],
        thickness: match extrusion {
            RibExtrusion::ParallelToSketch => np,
            RibExtrusion::NormalToSketch => np.cross(chord).normalized().or_refuse(KernelStage::Collect, "normalized")?,
        },
        chord,
        growth: along,
        quantum: 1e-7 * reach.max(1.0),
    };
    restamp_rib_faces(&mut fused, &sources, &names.feature, &frame);
    crate::feature_pipeline::features::common::stamp_derived_edge_names(&mut fused);
    Ok(fused)
}

/// Twice the part's bounding diagonal, measured from the chain too so a sketch
/// standing off the part still sweeps across it.  The rib's real extent is decided
/// by the cut in [`up_to_next`]; this only has to be generous.
fn sweep_reach(solid: &BrepSolid, chain: &[Vec3]) -> Result<f64, KernelRefusal> {
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut extend = |point: Vec3| {
        min = Vec3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z));
        max = Vec3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z));
    };
    for vertex in &solid.vertices {
        extend(vertex.point);
    }
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for row in &face.surface.control_points {
            for point in row {
                extend(point.point().or_refuse(KernelStage::Collect, "control_point")?);
            }
        }
    }
    for point in chain {
        extend(*point);
    }
    let diagonal = max.sub(min).length();
    if !(diagonal > 0.0) || !diagonal.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "target_extent",
            "rib: the target solid has no extent to grow the rib against",
        ));
    }
    Ok(diagonal * 2.0)
}

/// Points ON the chain, one per segment — where the rib starts, and so where the
/// search for its free-space piece begins.
///
/// It has to be the chain ITSELF, not the average of its vertices: for a bent
/// chain that average is off the chain entirely (an L's vertex centroid lands
/// exactly on the thickened ribbon's inner corner, a knife-edge the classifier
/// can only answer "on"), and a probe that starts on a boundary finds no piece.
fn chain_probe_seeds(profile: &[NurbsCurve]) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut seeds = Vec::with_capacity(profile.len());
    for curve in profile {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        seeds.push(curve.evaluate(start + (end - start) * 0.5).or_refuse(KernelStage::Collect, "evaluate")?);
    }
    if seeds.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_empty",
            "rib: profile has no points to grow from",
        ));
    }
    Ok(seeds)
}

/// **Parallel to Sketch**: the chain swept `reach` along the IN-PLANE direction
/// `along` gives a closed region inside the sketch plane; that region, offset to
/// −thickness/2 and extruded `thickness` along the plane normal, is the fin.
///
/// The loop must be simple, so a chain that doubles back across its own sweep
/// self-intersects here and the extrude refuses — the documented V1 limit.
fn parallel_slab(
    profile: &[NurbsCurve],
    vertices: &[Vec3],
    np: Vec3,
    along: Vec3,
    thickness: f64,
    reach: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let offset = along.scale(reach);
    let chain_start = vertices[0];
    let chain_end = *vertices.last().expect("chain has vertices");
    let mut region: Vec<NurbsCurve> = Vec::with_capacity(profile.len() * 2 + 2);
    for curve in profile {
        region.push(curve.clone());
    }
    region.push(make_line(chain_end, chain_end.add(offset)).or_refuse(KernelStage::Fragment, "make_line")?);
    for curve in profile.iter().rev() {
        region.push(super::extrude::translated_curve(&curve.reversed().or_refuse(KernelStage::Fragment, "reversed")?, offset)?);
    }
    region.push(make_line(chain_start.add(offset), chain_start).or_refuse(KernelStage::Fragment, "make_line")?);

    // Centre the thickness on the sketch plane: start half a thickness under it
    // and extrude a full thickness back through.
    let base = region
        .iter()
        .map(|curve| super::extrude::translated_curve(curve, np.scale(-thickness * 0.5)))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    extrude_profile_brep(&base, np, thickness).map_err(|error| {
        error.with_message(|error| {
            format!("rib: sweeping the profile inside its plane failed: {error}")
        })
    })
}

/// **Normal to Sketch**: the chain thickened INSIDE its own plane (miter-offset
/// ±thickness/2, straight caps across the two open ends → a closed thin loop),
/// extruded `reach` along `along` (the plane normal).
fn normal_slab(
    profile: &[NurbsCurve],
    np: Vec3,
    along: Vec3,
    thickness: f64,
    reach: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let half = thickness * 0.5;
    let left = offset_profile_segments(profile, np, half, false)
        .map_err(|error| error.with_message(|error| format!("rib: {error}")))?;
    let right = offset_profile_segments(profile, np, -half, false)
        .map_err(|error| error.with_message(|error| format!("rib: {error}")))?;
    let left_first = &left[0];
    let left_last = &left[left.len() - 1];
    let right_first = &right[0];
    let right_last = &right[right.len() - 1];
    let left_start = left_first.evaluate(left_first.domain().or_refuse(KernelStage::Fragment, "domain")?[0]).or_refuse(KernelStage::Fragment, "evaluate")?;
    let left_end = left_last.evaluate(left_last.domain().or_refuse(KernelStage::Fragment, "domain")?[1]).or_refuse(KernelStage::Fragment, "evaluate")?;
    let right_start = right_first.evaluate(right_first.domain().or_refuse(KernelStage::Fragment, "domain")?[0]).or_refuse(KernelStage::Fragment, "evaluate")?;
    let right_end = right_last.evaluate(right_last.domain().or_refuse(KernelStage::Fragment, "domain")?[1]).or_refuse(KernelStage::Fragment, "evaluate")?;
    let mut thin_loop: Vec<NurbsCurve> = Vec::with_capacity(left.len() + right.len() + 2);
    for curve in &left {
        thin_loop.push(curve.clone());
    }
    thin_loop.push(make_line(left_end, right_end).or_refuse(KernelStage::Fragment, "make_line")?);
    for curve in right.iter().rev() {
        thin_loop.push(curve.reversed().or_refuse(KernelStage::Fragment, "reversed")?);
    }
    thin_loop.push(make_line(right_start, left_start).or_refuse(KernelStage::Fragment, "make_line")?);
    extrude_profile_brep(&thin_loop, along, reach).map_err(|error| {
        error.with_message(|error| {
            format!("rib: extrude of the thickened profile failed: {error}")
        })
    })
}

/// SolidWorks' **Up To Next**, exactly: cut the over-long `slab` by the part and
/// keep the piece the sketch grew into.
///
/// `slab − solid` leaves the sweep's free-space pieces, and the boolean assembler
/// already groups a disconnected result into ONE SHELL PER PIECE (it unions faces
/// by shared edges), so the pieces are the result's shells. The piece containing
/// the sketch is the rib; anything past the part is a different piece and is
/// dropped — that, not a bounding box, is what makes the rib stop at the part's
/// own faces whatever shape they are.
///
/// `Ok(None)` means the sweep was entirely inside existing material: there is
/// nothing to add. A piece still running at `reach` never landed on anything, and
/// that is SolidWorks' documented failure ("if any portion of the solid feature
/// generated does not hit a Next face it fails").
fn up_to_next(
    solid: &BrepSolid,
    slab: &BrepSolid,
    seeds: &[Vec3],
    along: Vec3,
    reach: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let free = match boolean_operation(
        slab,
        solid,
        BooleanOperation::Subtract,
        &BooleanOptions::default(),
    ) {
        Ok(free) => free,
        // A subtract that refuses because the operands are disjoint means the
        // sweep never reached the part at all — the Up To Next failure, reported
        // as itself rather than as a boolean's internal complaint.
        Err(error) => {
            // ESSENTIAL REFUSAL (see the sibling below): the cut refusing because
            // the operands are disjoint IS "the rib met nothing". The cut's own
            // class rides through under the rib's text.
            return Err(error.with_message(|error| {
                format!(
                "rib: RIB_UP_TO_NEXT_UNBOUNDED — the rib never reaches the part, so it has \
                 nothing to stop against (SolidWorks' Up To Next requires every part of a rib \
                 to meet a face); check the rib's direction — the cut reported: {error}"
                )
            }))
        }
    };
    if free.shells.is_empty() {
        return Ok(None);
    }

    // Where the rib actually begins: the first point along the sweep from each
    // seed that is NOT already material. The chain can be drawn inside a wall, and
    // the rib is then the free space just beyond it.
    let classifier = SolidClassifier::new(solid, 1e-6).or_refuse(KernelStage::Select, "classifier")?;
    let steps = 64;
    let mut probes = Vec::new();
    for seed in seeds {
        for step in 1..=steps {
            let point = seed.add(along.scale(reach * step as f64 / steps as f64 * 0.5));
            if classifier.classify(point).or_refuse(KernelStage::Select, "classify")?.class == PointClass::Out {
                probes.push(point);
                break;
            }
        }
    }
    if probes.is_empty() {
        // Every sample along the sweep sits inside the part: nothing to add.
        return Ok(None);
    }

    let mut kept: Option<BrepSolid> = None;
    for shell in &free.shells {
        let piece = solid_from_shell(&free, shell);
        let grown_here = probes
            .iter()
            .map(|probe| classify_point(*probe, &piece, 1e-6))
            .collect::<Result<Vec<_>, String>>()
            .or_refuse(KernelStage::Select, "classify_point")?
            .into_iter()
            .any(|classification| classification.class == PointClass::In);
        if !grown_here {
            continue;
        }
        // SolidWorks' cardinal rule for Up To Next: a piece that is still going at
        // the end of the sweep never met a face.
        let overrun = piece
            .vertices
            .iter()
            .map(|vertex| vertex.point.sub(seeds[0]).dot(along))
            .fold(f64::NEG_INFINITY, f64::max);
        if overrun >= reach * 0.99 {
            // ESSENTIAL REFUSAL — do not "open" this one. Unlike a gate that
            // refuses data the machinery could already answer, this is the
            // feature's DEFINITION: SolidWorks' rib has one end condition, and
            // "if any portion of the solid feature generated does not hit a Next
            // face it fails" is the rule, not a limitation of ours. Accepting it
            // would ship a rib hanging in space with a far end at an arbitrary
            // sweep distance, which is exactly the bug this feature was reported
            // for. The right repair is always the rib's DIRECTION, never this
            // check.
            return Err(KernelRefusal::input(
                KernelStage::Select,
                "up_to_next_unbounded",
                "rib: RIB_UP_TO_NEXT_UNBOUNDED — part of the rib never lands on the part, so \
                 it has no face to stop against (SolidWorks' Up To Next requires the whole rib \
                 to terminate on a face). Turn the rib around with `direction`, or move the \
                 profile so its sweep meets the part",
            ));
        }
        kept = Some(match kept {
            None => piece,
            Some(previous) => boolean_operation(
                &previous,
                &piece,
                BooleanOperation::Union,
                &BooleanOptions::default(),
            )
            .map_err(|error| {
                error.with_message(|error| {
                    format!("rib: joining the rib's own pieces failed: {error}")
                })
            })?,
        });
    }
    Ok(kept)
}

/// One shell of `source` as a solid in its own right, carrying only the edges and
/// vertices its faces use — how a disconnected boolean result is taken apart into
/// the pieces the assembler already separated.
fn solid_from_shell(source: &BrepSolid, shell: &ShellRecord) -> BrepSolid {
    let edge_ids: std::collections::HashSet<u64> = shell
        .faces
        .iter()
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    let edges: Vec<EdgeRecord> = source
        .edges
        .iter()
        .filter(|edge| edge_ids.contains(&edge.id))
        .cloned()
        .collect();
    let vertex_ids: std::collections::HashSet<u64> = edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    BrepSolid {
        mass_properties_cache: Default::default(),
        id: source.id,
        vertices: source
            .vertices
            .iter()
            .filter(|vertex| vertex_ids.contains(&vertex.id))
            .cloned()
            .collect(),
        edges,
        shells: vec![shell.clone()],
        genus: 0,
    }
}

// ===========================================================================
// Face names
// ===========================================================================

/// What a rib calls its faces: the feature id every name starts with, and the
/// source name of each profile curve (`{sketchId}:G{gid}` for a sketch segment),
/// index-aligned with the profile.
///
/// | face | Parallel to Sketch | Normal to Sketch |
/// |---|---|---|
/// | the rib's two walls | `{id}:A` (−thickness axis), `{id}:B` (+) | `{id}:{segment}_A`, `{id}:{segment}_B` per segment |
/// | the rib's top (on the sketch) | `{id}:{segment}_TOP` per segment | `{id}:TOP` |
/// | its ends, at the chain's start and end | `{id}:START`, `{id}:END` | `{id}:START`, `{id}:END` |
/// | the far end (cut away by Up To Next) | `{id}:{segment}_FAR` | `{id}:FAR` |
///
/// The thickness axis is the sketch plane's authored normal for a Parallel rib,
/// and for a Normal rib the in-plane normal `plane normal × segment direction` —
/// the left of the chain seen from the side the normal points to.
///
/// A face the rib SPLITS is named by [`restamp_rib_faces`]: one fragment keeps
/// the name and the others are named after the rib and the side they are on.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RibNames {
    pub feature: String,
    pub segments: Vec<String>,
}

impl RibNames {
    /// Names for a caller with no segment identity to offer: `SEG{i}`.
    pub fn positional(feature: &str, count: usize) -> Self {
        Self {
            feature: feature.to_string(),
            segments: (0..count).map(|index| format!("SEG{index}")).collect(),
        }
    }
}

/// The slab's face names in `extrude_profile_brep`'s face order: the region's
/// sides in input order, then the base cap, then the far cap.
///
/// Parallel region: `[segments…, end line, far segments reversed…, start line]`,
/// base cap at −thickness/2 along the normal. Normal region: `[+offset
/// segments…, end cap, −offset segments reversed…, start cap]`, base cap on the
/// sketch plane.
fn slab_face_names(names: &RibNames, extrusion: RibExtrusion, count: usize) -> Vec<String> {
    let id = &names.feature;
    let segment = |index: usize, role: &str| format!("{id}:{}_{role}", names.segments[index]);
    let mut faces = Vec::with_capacity(2 * count + 4);
    let (near, far, base, top) = match extrusion {
        RibExtrusion::ParallelToSketch => ("TOP", "FAR", format!("{id}:A"), format!("{id}:B")),
        RibExtrusion::NormalToSketch => ("B", "A", format!("{id}:TOP"), format!("{id}:FAR")),
    };
    faces.extend((0..count).map(|index| segment(index, near)));
    faces.push(format!("{id}:END"));
    faces.extend((0..count).rev().map(|index| segment(index, far)));
    faces.push(format!("{id}:START"));
    faces.push(base);
    faces.push(top);
    faces
}

/// Leads every source token, so no authored or derived name can spell one.
const SOURCE_TOKEN: char = '\u{1}';
const TARGET_TOKEN: char = 'T';
const RIB_TOKEN: char = 'R';

/// A copy of `solid` whose faces carry `{SOURCE_TOKEN}{kind}{index}` instead of
/// their names, plus the names, index-aligned.
fn tokenised(solid: &BrepSolid, kind: char) -> (BrepSolid, Vec<Option<String>>) {
    let mut copy = solid.clone();
    let mut names = Vec::new();
    for face in copy.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        names.push(face.name.take());
        face.name = Some(format!("{SOURCE_TOKEN}{kind}{}", names.len() - 1));
    }
    (copy, names)
}

/// Where each token points: the part's own face names and the slab's roles.
struct FaceSources {
    part: Vec<Option<String>>,
    rib: Vec<String>,
}

impl FaceSources {
    /// `(kind, index)` of a token-named face. The booleans append `_n` to the
    /// later fragments of a split face; the digits after the kind end the index.
    fn parse(name: &str) -> Option<(char, usize)> {
        let mut chars = name.strip_prefix(SOURCE_TOKEN)?.chars();
        let kind = chars.next()?;
        let digits: String = chars.take_while(|c| c.is_ascii_digit()).collect();
        Some((kind, digits.parse().ok()?))
    }

    fn name(&self, kind: char, index: usize) -> Option<&str> {
        match kind {
            TARGET_TOKEN => self.part.get(index)?.as_deref(),
            RIB_TOKEN => self.rib.get(index).map(String::as_str),
            _ => None,
        }
    }

    /// A boolean's refusal text with every token put back as the name it stands for.
    fn detokenise(&self, text: &str) -> String {
        let mut out = String::with_capacity(text.len());
        let mut chars = text.chars().peekable();
        while let Some(c) = chars.next() {
            if c != SOURCE_TOKEN {
                out.push(c);
                continue;
            }
            let kind = chars.next();
            let mut digits = String::new();
            while let Some(digit) = chars.peek().copied().filter(char::is_ascii_digit) {
                digits.push(digit);
                chars.next();
            }
            match (kind, digits.parse::<usize>()) {
                (Some(kind), Ok(index)) => out.push_str(self.name(kind, index).unwrap_or("<unnamed>")),
                _ => {
                    out.extend(kind);
                    out.push_str(&digits);
                }
            }
        }
        out
    }
}

/// The rib's own frame, which orders the fragments of a split face: the
/// thickness axis first (which SIDE of the rib a fragment is on), then along
/// the chord from the chain's start, then along the growth direction.
struct RibFrame {
    origin: Vec3,
    thickness: Vec3,
    chord: Vec3,
    growth: Vec3,
    /// Coordinates closer than this are equal (a fragment's position is
    /// recomputed by every rebuild; its last bits are not its identity).
    quantum: f64,
}

/// Replace every source token on `fused` with the face's final name.
///
/// A source face that reaches the result as ONE face keeps its name. A face
/// that arrives in several fragments keeps its name on ONE of them and names
/// the rest after the rib, by position in the rib's own frame rather than by
/// the order the boolean met them or by their sizes (both change with edits
/// that have nothing to do with the rib — moving a hole from one half of a
/// floor to the other swaps which half is larger):
///
/// - a PART face split by the rib: its fragments are sorted by side (the rib's
///   A side, straddling, then its B side), then along the chord, then along the
///   growth direction. The first keeps the part's name; every other is
///   `{id}:{name}_{side}{k}`, `k` counting that side's fragments from 1 in the
///   same order — so a floor a rib crosses stays `Floor` on the A side and
///   becomes `{id}:Floor_B1` on the B side.
/// - one of the rib's OWN faces in several pieces (the part interrupts the rib):
///   along the chord, then the growth direction; the first keeps the name and
///   the others are `{name}_2`, `{name}_3`, …
///
/// A part face that had no name keeps none.
fn restamp_rib_faces(fused: &mut BrepSolid, sources: &FaceSources, feature: &str, frame: &RibFrame) {
    use std::collections::{BTreeMap, HashMap};
    let points: HashMap<u64, Vec3> = fused.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let ends: HashMap<u64, (u64, u64)> = fused
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
        .collect();
    let quantise = |value: f64| (value / frame.quantum).round() as i64;
    // (shell, face) positions per source, each with its sort key.
    let mut groups: BTreeMap<(char, usize), Vec<((i64, i64, i64), (usize, usize))>> = BTreeMap::new();
    for (shell_index, shell) in fused.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            let Some(source) = face.name.as_deref().and_then(FaceSources::parse) else {
                continue;
            };
            let mut ids: Vec<u64> = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter_map(|coedge| ends.get(&coedge.edge_id))
                .flat_map(|(start, end)| [*start, *end])
                .collect();
            ids.sort_unstable();
            ids.dedup();
            let mut centre = Vec3::default();
            for id in &ids {
                if let Some(point) = points.get(id) {
                    centre = centre.add(*point);
                }
            }
            let centre = centre.scale(1.0 / ids.len().max(1) as f64).sub(frame.origin);
            let key = (
                quantise(centre.dot(frame.thickness)),
                quantise(centre.dot(frame.chord)),
                quantise(centre.dot(frame.growth)),
            );
            groups.entry(source).or_default().push((key, (shell_index, face_index)));
        }
    }
    for ((kind, index), mut fragments) in groups {
        let original = sources.name(kind, index).map(str::to_string);
        let side = |key: &(i64, i64, i64)| match key.0.signum() {
            -1 => ('A', 0),
            0 => ('M', 1),
            _ => ('B', 2),
        };
        let named: Vec<((usize, usize), Option<String>)> = match &original {
            None => fragments.into_iter().map(|(_, at)| (at, None)).collect(),
            Some(name) if fragments.len() == 1 => vec![(fragments[0].1, Some(name.clone()))],
            Some(name) if kind == RIB_TOKEN => {
                fragments.sort_by_key(|(key, _)| (key.1, key.2, key.0));
                fragments
                    .into_iter()
                    .enumerate()
                    .map(|(rank, (_, at))| {
                        (at, Some(if rank == 0 { name.clone() } else { format!("{name}_{}", rank + 1) }))
                    })
                    .collect()
            }
            Some(name) => {
                fragments.sort_by_key(|(key, _)| (side(key).1, key.1, key.2, key.0));
                let mut per_side: BTreeMap<char, usize> = BTreeMap::new();
                fragments
                    .into_iter()
                    .enumerate()
                    .map(|(rank, (key, at))| {
                        let letter = side(&key).0;
                        let ordinal = per_side.entry(letter).or_insert(0);
                        *ordinal += 1;
                        let spelled = if rank == 0 {
                            name.clone()
                        } else {
                            format!("{feature}:{name}_{letter}{ordinal}")
                        };
                        (at, Some(spelled))
                    })
                    .collect()
            }
        };
        for ((shell_index, face_index), name) in named {
            fused.shells[shell_index].faces[face_index].name = name;
        }
    }
}
