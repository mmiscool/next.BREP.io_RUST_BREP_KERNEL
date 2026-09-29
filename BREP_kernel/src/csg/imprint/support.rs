use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Split every iso-line of `surface` along one direction and keep the range
/// [range[0], range[1]]. Knot insertion is exact, so the result is the same
/// geometry restricted to the range and its control hull bounds it.
pub(super) fn restrict_direction(
    surface: &NurbsSurface,
    range: [f64; 2],
    direction_u: bool,
) -> Result<NurbsSurface, KernelRefusal> {
    let (knots, degree, domain) = if direction_u {
        (
            &surface.knots_u,
            surface.degree_u,
            surface
                .domain_u()
                .or_refuse(KernelStage::Intersect, "domain_u")?,
        )
    } else {
        (
            &surface.knots_v,
            surface.degree_v,
            surface
                .domain_v()
                .or_refuse(KernelStage::Intersect, "domain_v")?,
        )
    };
    // NurbsCurve::split refuses parameters within its absolute knot identity
    // tolerance of the domain ends (same guard as edge_subcurve). Mirror the
    // single source (curve.rs KNOT_IDENTITY_TOL) rather than a bare literal.
    let guard = ((domain[1] - domain[0]) * KNOT_IDENTITY_TOL).max(2e-9);
    let mut curves: Vec<NurbsCurve> = if direction_u {
        (0..surface.control_points[0].len())
            .map(|column| {
                NurbsCurve::new(
                    degree,
                    knots.clone(),
                    surface
                        .control_points
                        .iter()
                        .map(|row| row[column])
                        .collect(),
                )
            })
            .collect::<Result<_, _>>()
            .or_refuse(KernelStage::Intersect, "csg.imprint.support")?
    } else {
        surface
            .control_points
            .iter()
            .map(|row| NurbsCurve::new(degree, knots.clone(), row.clone()))
            .collect::<Result<_, _>>()
            .or_refuse(KernelStage::Intersect, "csg.imprint.support")?
    };
    if range[1] < domain[1] - guard && range[1] > domain[0] + guard {
        curves = curves
            .into_iter()
            .map(|curve| curve.split(range[1]).map(|(left, _)| left))
            .collect::<Result<_, _>>()
            .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
    }
    if range[0] > domain[0] + guard && range[0] < domain[1] - guard {
        curves = curves
            .into_iter()
            .map(|curve| curve.split(range[0]).map(|(_, right)| right))
            .collect::<Result<_, _>>()
            .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
    }
    let new_knots = curves[0].knots.clone();
    if direction_u {
        let rows = curves[0].control_points.len();
        let control = (0..rows)
            .map(|row| {
                curves
                    .iter()
                    .map(|curve| curve.control_points[row])
                    .collect()
            })
            .collect();
        NurbsSurface::new(
            degree,
            surface.degree_v,
            new_knots,
            surface.knots_v.clone(),
            control,
        )
        .or_refuse(KernelStage::Intersect, "NurbsSurface::new")
    } else {
        let control = curves
            .iter()
            .map(|curve| curve.control_points.clone())
            .collect();
        NurbsSurface::new(
            surface.degree_u,
            degree,
            surface.knots_u.clone(),
            new_knots,
            control,
        )
        .or_refuse(KernelStage::Intersect, "NurbsSurface::new")
    }
}

/// A CLOSED direction's own far span, carried across the seam so a trim drawn
/// PAST the domain lives in ONE window with the rest of the face's trim.
///
/// The strip is the carrier's own control rows with their knots shifted by the
/// period, so on the lifted surface `S(u) = S(u ± period)` outside the original
/// domain: the wrap `evaluate_extended` reads, and the convention an importer
/// drawing a pcurve past the domain used. The original domain's knots and
/// control net are kept VERBATIM (the [`NurbsSurface::extend_natural`] rule),
/// so evaluation inside it is bit-identical and every pcurve already on the
/// face stays valid with no remap.
///
/// `None` when the direction is not closed, the strip would reach a whole
/// period, or the carrier does not close on its own CONTROL rows. The join
/// keeps one of the two boundary control lines and drops the other, so a
/// carrier whose lines merely agree geometrically (`closed_directions` reads
/// 1e-6) would take that disagreement into the strip as geometry that is not
/// the carrier's. The identity bar is the control net's own: `1e-12` of the
/// face's 3D extent, the same shape as `align_collapsed_endpoints`' pole band.
/// The section fit is the backstop — it measures every marched run against the
/// UNLIFTED carrier, so a strip that is not the carrier reads there.
pub(super) fn seam_lifted_carrier(
    surface: &NurbsSurface,
    direction_u: bool,
    delta: f64,
    low_side: bool,
) -> Option<NurbsSurface> {
    let [d0, d1] = if direction_u {
        surface.domain_u().ok()?
    } else {
        surface.domain_v().ok()?
    };
    let period = d1 - d0;
    if !(delta > 0.0) || delta >= period || !period.is_finite() {
        return None;
    }
    let degree = if direction_u {
        surface.degree_u
    } else {
        surface.degree_v
    };
    let rows = &surface.control_points;
    let (rows_count, columns_count) = (rows.len(), rows[0].len());
    // The two boundary control lines the join welds into one.
    let line_pairs: Vec<(Vec4, Vec4)> = if direction_u {
        (0..columns_count)
            .map(|column| (rows[0][column], rows[rows_count - 1][column]))
            .collect()
    } else {
        (0..rows_count)
            .map(|row| (rows[row][0], rows[row][columns_count - 1]))
            .collect()
    };
    let point_of = |control: Vec4| -> Option<Vec3> { control.point().ok() };
    let anchor = point_of(rows[0][0])?;
    let mut extent = 0.0f64;
    for control in rows.iter().flatten() {
        extent = extent.max(point_of(*control)?.sub(anchor).length());
    }
    let identity = 1e-12 * (1.0 + extent);
    for (first, last) in &line_pairs {
        if point_of(*first)?.sub(point_of(*last)?).length() > identity {
            return None;
        }
        let weight_scale = first.w.abs().max(last.w.abs()).max(f64::MIN_POSITIVE);
        if (first.w - last.w).abs() / weight_scale > 1e-12 {
            return None;
        }
    }
    let strip_range = if low_side {
        [d1 - delta, d1]
    } else {
        [d0, d0 + delta]
    };
    let mut strip = restrict_direction(surface, strip_range, direction_u).ok()?;
    let shift = if low_side { -period } else { period };
    if direction_u {
        for knot in &mut strip.knots_u {
            *knot += shift;
        }
    } else {
        for knot in &mut strip.knots_v {
            *knot += shift;
        }
    }
    // Two clamped pieces meeting at one shared control line join exactly by
    // dropping one copy of the junction knot (multiplicity `degree`, C0) and
    // one of the two coincident control lines.
    let join_knots = |low: &[f64], high: &[f64]| -> Vec<f64> {
        let junction = low[low.len() - 1];
        let mut knots = low[..low.len() - degree - 1].to_vec();
        knots.extend(std::iter::repeat_n(junction, degree));
        knots.extend_from_slice(&high[degree + 1..]);
        knots
    };
    let (low_surface, high_surface) = if low_side {
        (&strip, surface)
    } else {
        (surface, &strip)
    };
    let control = if direction_u {
        let low_rows = &low_surface.control_points;
        low_rows[..low_rows.len() - 1]
            .iter()
            .cloned()
            .chain(high_surface.control_points.iter().cloned())
            .collect::<Vec<_>>()
    } else {
        low_surface
            .control_points
            .iter()
            .zip(&high_surface.control_points)
            .map(|(low_row, high_row)| {
                low_row[..low_row.len() - 1]
                    .iter()
                    .copied()
                    .chain(high_row.iter().copied())
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>()
    };
    let (knots_u, knots_v) = if direction_u {
        (
            join_knots(&low_surface.knots_u, &high_surface.knots_u),
            surface.knots_v.clone(),
        )
    } else {
        (
            surface.knots_u.clone(),
            join_knots(&low_surface.knots_v, &high_surface.knots_v),
        )
    };
    NurbsSurface::new(
        surface.degree_u,
        surface.degree_v,
        knots_u,
        knots_v,
        control,
    )
    .ok()
}

/// The carrier restricted (by exact knot-insertion splitting) to the uv
/// bounding box of the trim pcurves. The pcurve control hulls bound the
/// loops, the loops bound the trimmed region, and splitting preserves the
/// global parameterization, so the result is the same surface over a window
/// that still contains the whole trimmed region — usable interchangeably
/// with the carrier for bounding, seeding, and marching. Returns None when
/// the trims span the whole carrier (nothing to gain) or the restriction
/// fails (callers keep the carrier, which is always conservative).
///
/// A trim drawn PAST a closed direction's domain does not fit in one in-domain
/// interval: it covers the wrapped image of its overhang as well. The window is
/// then taken on a carrier [lifted](seam_lifted_carrier) across the seam — the
/// trim's own frame, unwrapped, never clamped — and the window's `lifted` flag
/// says so, because every uv the imprint reads for that face (the clip's
/// containment, the section's pcurves) must be read in that frame and not in
/// the carrier's. Escape hatch `BREP_MARCH_SEAM_LIFT=0` restores the clamp.
pub(super) struct MarchWindow {
    pub(super) surface: NurbsSurface,
    /// The window's frame is the trim's, a period off the carrier's on one
    /// side of the seam. `false` means the carrier's own parameterization,
    /// which is what every window before 2026-09-15 was.
    pub(super) lifted: bool,
}

/// Every LIFTED face's chart, by key — the imprint's post-passes rebuild
/// pcurves and must draw them in the frame the mint used. A face absent here
/// is read on its own carrier, which is every face whose trim sits inside its
/// domain.
pub(super) type FaceCharts<'a> = HashMap<FaceKey, &'a NurbsSurface>;

/// The frame a face's uv are read in: its chart where the window was lifted,
/// its carrier otherwise.
pub(super) fn chart_of<'a>(
    charts: &FaceCharts<'a>,
    key: FaceKey,
    face: &'a FaceRecord,
) -> &'a NurbsSurface {
    charts.get(&key).copied().unwrap_or(&face.surface)
}

pub(super) fn restricted_carrier(face: &FaceRecord) -> Option<MarchWindow> {
    let [u0, u1] = face.surface.domain_u().ok()?;
    let [v0, v1] = face.surface.domain_v().ok()?;
    let mut u_low = f64::INFINITY;
    let mut u_high = f64::NEG_INFINITY;
    let mut v_low = f64::INFINITY;
    let mut v_high = f64::NEG_INFINITY;
    let mut found = false;
    for coedge in face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        for control in &coedge.pcurve.control_points {
            let point = control.point().ok()?;
            u_low = u_low.min(point.x);
            u_high = u_high.max(point.x);
            v_low = v_low.min(point.y);
            v_high = v_high.max(point.y);
            found = true;
        }
    }
    if !found {
        return None;
    }
    // CENSUS (`BREP_DEBUG_OVERHANG=1`): a trim drawn past a CLOSED direction's
    // domain covers the wrapped image of its overhang too, and the clamp below
    // keeps only the in-domain part — so the section's stretch in the overhang
    // is never marched. One line per face and direction, for the population.
    if std::env::var_os("BREP_DEBUG_OVERHANG").is_some() {
        if let Ok((closed_u, closed_v)) = face.surface.closed_directions() {
            for (axis, closed, domain, low, high) in [
                (0usize, closed_u, [u0, u1], u_low, u_high),
                (1, closed_v, [v0, v1], v_low, v_high),
            ] {
                let below = (domain[0] - low).max(0.0);
                let above = (high - domain[1]).max(0.0);
                if !closed || (below <= 0.0 && above <= 0.0) {
                    continue;
                }
                let period = domain[1] - domain[0];
                eprintln!(
                    "overhang: face {} axis {} domain [{:.6},{:.6}] hull [{:.9},{:.9}] below {:.3e} above {:.3e} span/period {:.6} discards {}",
                    face.id,
                    if axis == 0 { "u" } else { "v" },
                    domain[0],
                    domain[1],
                    low,
                    high,
                    below,
                    above,
                    (high - low) / period,
                    if high - low < period { "YES" } else { "no" }
                );
            }
        }
    }
    // Generous margin: the window must strictly contain the trims so that
    // trim-boundary crossings (which the clip stage locates) stay interior.
    let u_margin = ((u1 - u0) * 1e-3).max(2e-9);
    let v_margin = ((v1 - v0) * 1e-3).max(2e-9);
    // THE OVERHANG: a trim drawn past a CLOSED direction's domain and spanning
    // less than a period has ONE unwrapped interval and no in-domain one. The
    // carrier is lifted across the seam by exactly the overhang (plus the same
    // margin), and the window is then taken on the lifted carrier by the
    // ordinary restriction below: the trim's own frame, never clamped. A trim
    // spanning a period or more is the wrap guard's case and keeps the full
    // period. Escape hatch `BREP_MARCH_SEAM_LIFT=0`.
    let mut lifted_surface: Option<NurbsSurface> = None;
    let mut lifted_axis: Option<bool> = None;
    if std::env::var("BREP_MARCH_SEAM_LIFT").as_deref() != Ok("0") {
        let (closed_u, closed_v) = face.surface.closed_directions().ok()?;
        for (direction_u, closed, domain, low, high, margin) in [
            (true, closed_u, [u0, u1], u_low, u_high, u_margin),
            (false, closed_v, [v0, v1], v_low, v_high, v_margin),
        ] {
            let below = (domain[0] - low).max(0.0);
            let above = (high - domain[1]).max(0.0);
            // An overhang the SPLIT could not cut is parametric round-off, not
            // trim: `restrict_direction`'s own guard, the absolute knot
            // identity tolerance over the domain. Measured over the boolean
            // fuzz corpus, the two populations are eight orders apart — a
            // hull end that overshoots its domain by 1e-16 .. 1.3e-10 (52 of
            // 57 rows) against the helmet's 1.807e-1 — and lifting the
            // round-off ones re-marched their sections for nothing: 8 rows of
            // `21_band_rim_containment` and `27_corner_chord_duplicate_t634`
            // moved in their last digits with no topology change.
            let guard = ((domain[1] - domain[0]) * KNOT_IDENTITY_TOL).max(2e-9);
            if !closed
                || below.max(above) <= guard
                || high - low >= domain[1] - domain[0]
            {
                continue;
            }
            let low_side = below > 0.0;
            let delta = below.max(above) + margin;
            if let Some(lift) =
                seam_lifted_carrier(&face.surface, direction_u, delta, low_side)
            {
                lifted_surface = Some(lift);
                lifted_axis = Some(direction_u);
                break;
            }
        }
    }
    let carrier = lifted_surface.as_ref().unwrap_or(&face.surface);
    let [u0, u1] = carrier.domain_u().ok()?;
    let [v0, v1] = carrier.domain_v().ok()?;
    u_low = (u_low - u_margin).max(u0);
    u_high = (u_high + u_margin).min(u1);
    v_low = (v_low - v_margin).max(v0);
    v_high = (v_high + v_margin).min(v1);
    if u_high <= u_low || v_high <= v_low {
        return None;
    }
    let mut full_u = u_low <= u0 + u_margin && u_high >= u1 - u_margin;
    let mut full_v = v_low <= v0 + v_margin && v_high >= v1 - v_margin;
    // WRAPPED-BAND HULL GUARD: on a CLOSED direction the pcurve hull does NOT
    // necessarily bound the material — a band face whose two rims sit at the
    // hull edges can have its material in the WRAPPED COMPLEMENT of the hull
    // window (trial 35's torus face: loops at v=0.25/1.0, material v∈[0,0.25]).
    // Restricting the march surface to such a hull excludes every interior
    // material point AND breaks closure, so the SSI trace terminates at the
    // rim as a false "boundary" and the section over the band is never
    // marched. The complement strip of the full hull contains no loop curves
    // at all, so it is uniformly material or uniformly not: ONE sample at its
    // middle decides exactly. If that sample is Inside, the hull is invalid in
    // that direction — keep the full period (the unrestricted carrier is
    // always conservative). Escape hatch: BREP_RESTRICT_WRAP_GUARD=0.
    // A LIFTED direction is not asked: the lift exists because the trim's own
    // hull is the material band, drawn unwrapped, so the complement carries no
    // material by construction — and the sample's parameter would be read in
    // the carrier's frame while the window is in the trim's.
    if std::env::var("BREP_RESTRICT_WRAP_GUARD").as_deref() != Ok("0") {
        let (mut closed_u, mut closed_v) = face.surface.closed_directions().ok()?;
        match lifted_axis {
            Some(true) => closed_u = false,
            Some(false) => closed_v = false,
            None => {}
        }
        let sample_inside = |uv: crate::arrangement::Vec2| -> bool {
            matches!(
                parameter_point_in_face(face, uv, 1e-9),
                Ok(PolygonClass::Inside)
            )
        };
        if closed_u && !full_u {
            let period = u1 - u0;
            let gap = period - (u_high - u_low);
            let mut mid = u_high + gap / 2.0;
            if mid > u1 {
                mid -= period;
            }
            if sample_inside(crate::arrangement::Vec2 {
                x: mid,
                y: (v_low + v_high) / 2.0,
            }) {
                full_u = true;
            }
        }
        if closed_v && !full_v {
            let period = v1 - v0;
            let gap = period - (v_high - v_low);
            let mut mid = v_high + gap / 2.0;
            if mid > v1 {
                mid -= period;
            }
            if sample_inside(crate::arrangement::Vec2 {
                x: (u_low + u_high) / 2.0,
                y: mid,
            }) {
                full_v = true;
            }
        }
    }
    let lifted = lifted_axis.is_some();
    if full_u && full_v {
        // A lifted carrier is itself the window — the strip past the seam is
        // the whole point, even where the hull narrows nothing further.
        return lifted_surface.map(|surface| MarchWindow { surface, lifted });
    }
    let restricted_u = if full_u {
        None
    } else {
        Some(restrict_direction(carrier, [u_low, u_high], true).ok()?)
    };
    let surface = if full_v {
        restricted_u?
    } else {
        let base = restricted_u.as_ref().unwrap_or(carrier);
        restrict_direction(base, [v_low, v_high], false).ok()?
    };
    Some(MarchWindow { surface, lifted })
}

pub(super) fn face_bounds(
    face: &FaceRecord,
    restricted: Option<&NurbsSurface>,
    edges: &HashMap<(u8, u64), &EdgeRecord>,
    operand: u8,
    tolerance: f64,
) -> Result<Aabb, KernelRefusal> {
    let [u0, u1] = face
        .surface
        .domain_u()
        .or_refuse(KernelStage::Intersect, "domain_u")?;
    let [v0, v1] = face
        .surface
        .domain_v()
        .or_refuse(KernelStage::Intersect, "domain_v")?;
    let origin = face
        .surface
        .evaluate((u0 + u1) / 2.0, (v0 + v1) / 2.0)
        .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
    let normal = face
        .surface
        .normal((u0 + u1) / 2.0, (v0 + v1) / 2.0)
        .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
    let surface_controls = face
        .surface
        .control_points
        .iter()
        .flatten()
        .map(|control| control.point())
        .collect::<Result<Vec<_>, _>>()
        .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
    let scale = surface_controls
        .iter()
        .map(|point| point.sub(origin).length())
        .fold(1.0f64, f64::max);
    let plane_tolerance = (tolerance * 100.0).max(1e-7) * scale;
    if surface_controls
        .iter()
        .any(|point| point.sub(origin).dot(normal).abs() > plane_tolerance)
    {
        // Curved carrier: bound the trimmed uv sub-patch when the trims
        // cover only part of it (a revolve wall's full carrier box spans
        // the whole ring and pairs with everything).
        if let Some(restricted) = restricted {
            return Ok(Aabb::from_surface_controls(restricted)
                .or_refuse(KernelStage::Intersect, "from_surface_controls")?
                .expanded(plane_tolerance));
        }
        return Aabb::from_surface_controls(&face.surface)
            .or_refuse(KernelStage::Intersect, "from_surface_controls");
    }

    // A planar trimmed region lies inside the convex hull of its boundary.
    // Bounding its exact trimmed edge curves is therefore conservative and
    // substantially tighter than the often much larger carrier patch.
    let mut bounds = Aabb::empty();
    let mut found = false;
    for coedge in face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        let edge = edges.get(&(operand, coedge.edge_id)).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Intersect,
                "imprint.support",
                format!("missing edge {} on operand {operand}", coedge.edge_id),
            )
        })?;
        let curve = edge_subcurve(edge)?;
        for control in &curve.control_points {
            bounds.include_point(control.point().or_refuse(KernelStage::Intersect, "point")?);
            found = true;
        }
    }
    if !found {
        return Aabb::from_surface_controls(&face.surface)
            .or_refuse(KernelStage::Intersect, "from_surface_controls");
    }
    Ok(bounds.expanded(plane_tolerance))
}

/// The parameter seam of a closed revolution carrier, as the plane it lies in
/// and the half of that plane it occupies: the u = 0 meridian is the half-plane
/// through the axis on `frame.x_axis`'s side, and a torus's v = 0 circle is the
/// outer half of its equatorial plane. An analytic carrier keeps every pcurve
/// on its own period (`build_interpolant` clamps it there), so a trim cannot
/// run across one of these; a section that crosses one inside a face needs a
/// vertex on it, whether or not the face's own seam EDGE sits there.
pub(super) struct CarrierSeam {
    origin: Vec3,
    normal: Vec3,
    half: SeamHalf,
}

enum SeamHalf {
    Toward(Vec3),
    Beyond { axis: Vec3, radius: f64 },
}

impl CarrierSeam {
    pub(super) fn of(surface: &NurbsSurface) -> Result<Vec<CarrierSeam>, KernelRefusal> {
        let Some(analytic) = surface.analytic() else {
            return Ok(Vec::new());
        };
        let Some(frame) = analytic.frame() else {
            return Ok(Vec::new());
        };
        let (closed_u, closed_v) = surface
            .closed_directions()
            .or_refuse(KernelStage::Intersect, "closed_directions")?;
        let mut seams = Vec::new();
        if closed_u {
            seams.push(CarrierSeam {
                origin: frame.origin,
                normal: frame.y_axis,
                half: SeamHalf::Toward(frame.x_axis),
            });
        }
        if let (true, crate::AnalyticSurface::Torus { major_radius, .. }) = (closed_v, &analytic) {
            seams.push(CarrierSeam {
                origin: frame.origin,
                normal: frame.axis,
                half: SeamHalf::Beyond {
                    axis: frame.axis,
                    radius: *major_radius,
                },
            });
        }
        Ok(seams)
    }

    /// Signed distance from the seam's plane.
    pub(super) fn offset(&self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.normal)
    }

    /// Whether `point`'s foot on the seam's plane lies on the seam's half of it.
    pub(super) fn on_seam_half(&self, point: Vec3) -> bool {
        let relative = point.sub(self.origin);
        match self.half {
            SeamHalf::Toward(direction) => relative.dot(direction) > 0.0,
            SeamHalf::Beyond { axis, radius } => {
                relative.sub(axis.scale(relative.dot(axis))).length() > radius
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct TaggedFace<'a> {
    pub(super) operand: u8,
    pub(super) face: &'a FaceRecord,
    /// The face's [lifted](seam_lifted_carrier) carrier, where its trim is
    /// drawn past a closed direction's domain. Set by the driver from the
    /// face's own march window; `None` on every face whose trim sits inside
    /// its domain, which is all but the overhanging ones.
    pub(super) lifted: Option<&'a NurbsSurface>,
}

impl<'a> TaggedFace<'a> {
    pub(super) fn key(self) -> FaceKey {
        FaceKey {
            operand: self.operand,
            face_id: self.face.id,
        }
    }

    /// THE FRAME EVERY uv OF THIS FACE IS READ IN. For a face whose trim sits
    /// inside its domain this is the carrier itself, so every reading is what
    /// it was before the lift existed, to the bit. For a face whose trim is
    /// drawn past a closed direction's domain it is the lifted carrier, whose
    /// parameters are the trim's own: a section marched in the overhang then
    /// carries the parameters the face's loops are drawn in, and the
    /// containment tests and pcurve builds that read them need no period image.
    pub(super) fn chart(self) -> &'a NurbsSurface {
        self.lifted.unwrap_or(&self.face.surface)
    }
}

pub(super) fn faces(solid: &BrepSolid, operand: u8) -> Vec<TaggedFace<'_>> {
    solid
        .shells
        .iter()
        .flat_map(|shell| shell.faces.iter())
        .map(|face| TaggedFace { operand, face, lifted: None })
        .collect()
}

pub(super) fn edge_map<'a>(
    solid_a: &'a BrepSolid,
    solid_b: &'a BrepSolid,
) -> HashMap<(u8, u64), &'a EdgeRecord> {
    solid_a
        .edges
        .iter()
        .map(|edge| ((0, edge.id), edge))
        .chain(solid_b.edges.iter().map(|edge| ((1, edge.id), edge)))
        .collect()
}

pub(super) fn face_edges<'a>(
    face: TaggedFace<'_>,
    edges: &HashMap<(u8, u64), &'a EdgeRecord>,
) -> Result<Vec<&'a EdgeRecord>, KernelRefusal> {
    let mut seen = HashSet::default();
    let mut result = Vec::new();
    for coedge in face
        .face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        if seen.insert(coedge.edge_id) {
            result.push(*edges.get(&(face.operand, coedge.edge_id)).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Intersect,
                    "imprint.support",
                    format!("face {} references missing edge", face.face.id),
                )
            })?);
        }
    }
    Ok(result)
}

pub(super) fn edge_subcurve(edge: &EdgeRecord) -> Result<NurbsCurve, KernelRefusal> {
    let mut curve = edge.curve.clone();
    let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot identity
    // tolerance (curve.rs KNOT_IDENTITY_TOL) of the domain ends; the skip
    // epsilon must cover that or a near-edge trim parameter slips past the
    // guard and errors.
    let epsilon = (KNOT_IDENTITY_TOL * (end - start)).max(2e-9);
    if edge.t0 > start + epsilon && edge.t0 < end - epsilon {
        curve = curve
            .split(edge.t0)
            .or_refuse(KernelStage::Intersect, "split")?
            .1;
    }
    let domain = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    if edge.t1 < domain[1] - epsilon && edge.t1 > domain[0] + epsilon {
        curve = curve
            .split(edge.t1)
            .or_refuse(KernelStage::Intersect, "split")?
            .0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && edge.t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in imprint edge_subcurve");
    }
    Ok(curve)
}

pub(super) fn curve_length_rough(curve: &NurbsCurve) -> Result<f64, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let mut previous = curve
        .evaluate(start)
        .or_refuse(KernelStage::Intersect, "evaluate")?;
    let mut length = 0.0;
    for index in 1..=8 {
        let point = curve
            .evaluate(start + (end - start) * index as f64 / 8.0)
            .or_refuse(KernelStage::Intersect, "evaluate")?;
        length += point.sub(previous).length();
        previous = point;
    }
    Ok(length)
}

pub(super) fn parameter_tolerance(
    curve: &NurbsCurve,
    parameter: f64,
    tolerance: f64,
) -> Result<f64, KernelRefusal> {
    let tangent = curve
        .derivatives(parameter, 1)
        .or_refuse(KernelStage::Intersect, "derivatives")?[1]
        .length();
    Ok(tolerance / tangent.max(1e-12))
}

pub(super) fn snap_curve_ends(
    curve: &NurbsCurve,
    start: Vec3,
    end: Vec3,
) -> Result<NurbsCurve, KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let p0 = curve
        .evaluate(t0)
        .or_refuse(KernelStage::Intersect, "evaluate")?;
    let p1 = curve
        .evaluate(t1)
        .or_refuse(KernelStage::Intersect, "evaluate")?;
    let cap = 5e-3 * (1.0 + p0.length());
    let d0 = p0.sub(start).length();
    let d1 = p1.sub(end).length();
    if (d0 <= 1e-12 && d1 <= 1e-12) || d0 > cap || d1 > cap {
        return Ok(curve.clone());
    }
    let mut controls = curve.control_points.clone();
    if d0 > 1e-12 {
        controls[0] = Vec4::from_point(start, controls[0].w);
    }
    let last = controls.len() - 1;
    if d1 > 1e-12 {
        controls[last] = Vec4::from_point(end, controls[last].w);
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
        .or_refuse(KernelStage::Intersect, "NurbsCurve::new")
}

pub(super) fn curve_lies_on_surface(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    // This curve is being offered as lying on another carrier. A short
    // grazing contact is not a shared section, and translating the operands
    // must not enlarge that geometric allowance.
    let carrier_tolerance = tolerance.max(1e-7);
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let point = curve
            .evaluate(start + (end - start) * fraction)
            .or_refuse(KernelStage::Intersect, "evaluate")?;
        if project_point_to_surface(surface, point)
            .or_refuse(KernelStage::Intersect, "project_point_to_surface")?
            .distance
            > carrier_tolerance
        {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The spans of an operand edge's `curve` that lie on the other operand's
/// `face`, for the boundary-curve exchange: the whole curve when it lies on
/// the face's surface ([`curve_lies_on_surface`]), and otherwise each span
/// between the curve's crossings with the face's boundary edges that does.
///
/// The surface is a bounded patch, so a curve on its carrier that runs past
/// the patch does not lie on it, and the whole-curve test refused the part
/// that did. That part is a section: the flush partial torus's equator circles
/// lie in the box's top plane for 281 degrees and over the top face for the
/// last 11, and the march of that plane against the torus skips the same
/// section because it rides the torus's own edge. With neither lane minting
/// it, the union's shell stood open on five edges. Given the edges already
/// split where the imprint's sections meet them, the whole-curve test passed
/// on the sub-edge and the union built, so the answer depended on what the
/// input happened to have split.
///
/// A span that leaves the face crosses the face's boundary there, so the
/// crossings are the only span ends a curve can have short of its own ends,
/// and each span is held to the whole-curve test with its bar. The crossings
/// are `process_curve`'s own search, with its split guard and its overlap
/// skip, so the call that exchanges a span finds the crossing again at the
/// span's end; a tangential hit within its own precision of the curve's end is
/// that end. A boundary edge whose control hull stands farther than that
/// search's reach from the curve's is not searched. A curve that meets the
/// surface only where it lifts off inside the face has no crossing there and
/// is not exchanged, as before. `whole_only` is the caller's answer to "does an
/// exact lane already mint this pair's section": where one does, a span would
/// be a second copy of it. Escape hatch for tamper-verification:
/// `BREP_LIES_ON_SPANS=0` exchanges whole curves only.
pub(super) fn curve_spans_on_face(
    curve: NurbsCurve,
    face: &FaceRecord,
    face_edges: &[&EdgeRecord],
    tolerance: f64,
    whole_only: bool,
) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    if curve_lies_on_surface(&curve, &face.surface, tolerance)? {
        return Ok(vec![curve]);
    }
    if whole_only || std::env::var("BREP_LIES_ON_SPANS").as_deref() == Ok("0") {
        return Ok(Vec::new());
    }
    let hull = |curve: &NurbsCurve| -> Result<Aabb, KernelRefusal> {
        let points = curve
            .control_points
            .iter()
            .map(|point| point.point())
            .collect::<Result<Vec<_>, _>>()
            .or_refuse(KernelStage::Intersect, "control point")?;
        Ok(Aabb::from_points(points))
    };
    let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let split_tolerance = 2e-9f64.max((end - start) * KNOT_IDENTITY_TOL);
    let overlap_limit = tolerance.max(1e-5);
    const CROSSING_TOLERANCE: f64 = 1e-5;
    let start_point = curve.evaluate(start).or_refuse(KernelStage::Intersect, "evaluate")?;
    let end_point = curve.evaluate(end).or_refuse(KernelStage::Intersect, "evaluate")?;
    let curve_hull = hull(&curve)?;
    let mut crossings = Vec::new();
    for edge in face_edges {
        if edge.degenerate
            || !curve_hull.intersects(hull(&edge.curve)?, CROSSING_TOLERANCE)
            || curve_overlaps_edge(&curve, edge, overlap_limit)?
        {
            continue;
        }
        let hits = intersect_curves(&curve, &edge.curve, CROSSING_TOLERANCE)
            .map_err(|error| format!("imprint span crossing on edge {}: {error}", edge.id))
            .or_refuse(KernelStage::Intersect, "csg.imprint.support")?;
        for hit in hits {
            if hit.t < edge.t0 - 1e-9
                || hit.t > edge.t1 + 1e-9
                || hit.s <= start + split_tolerance
                || hit.s >= end - split_tolerance
            {
                continue;
            }
            // A tangential hit is only placed to `intersect_curves`' spatial
            // precision for one, √tolerance · (1 + |p|). Within that of the
            // curve's own end it is that end's junction: where a curve leaves a
            // corner tangent to one of the corner's edges (a flange's window
            // edge along a bend's inner arc) the hit lands 1.4e-6 off the corner,
            // and a span cut there started off it. A transversal hit is placed
            // to the search's tolerance and stays a crossing.
            if hit.tangential {
                let point = curve.evaluate(hit.s).or_refuse(KernelStage::Intersect, "evaluate")?;
                let precision = CROSSING_TOLERANCE.sqrt() * (1.0 + point.length());
                if point.sub(start_point).length() <= precision || point.sub(end_point).length() <= precision {
                    continue;
                }
            }
            crossings.push(hit.s);
        }
    }
    if crossings.is_empty() {
        return Ok(Vec::new());
    }
    crossings.sort_by(f64::total_cmp);
    crossings.dedup_by(|a, b| (*a - *b).abs() <= split_tolerance);
    let crossing_count = crossings.len();
    let mut spans = Vec::new();
    let mut rest = curve;
    for parameter in crossings {
        let (left, right) = rest.split(parameter).or_refuse(KernelStage::Intersect, "split")?;
        if curve_lies_on_surface(&left, &face.surface, tolerance)? {
            spans.push(left);
        }
        rest = right;
    }
    if curve_lies_on_surface(&rest, &face.surface, tolerance)? {
        spans.push(rest);
    }
    if std::env::var("BREP_DEBUG_PAIRS").is_ok() {
        eprintln!("lies-on spans: face {} {crossing_count} crossing(s), {} span(s) on it", face.id, spans.len());
    }
    Ok(spans)
}

/// Which predicate admits a direction of a carrier into the exact lane of
/// [`planar_linear_iso_intersection`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum IsoGate {
    /// The count proxy: degree 1 with exactly two control rows across the
    /// direction. One knot insertion sends the same surface to the marcher.
    Count,
    /// What the lane consumes: every ruling across the direction is a straight
    /// segment traced at constant speed ([`rulings_are_straight_and_affine`]).
    Geometry,
}

/// The gate the boolean runs: the geometry the lane consumes. The count proxy
/// sent a knot-refined or degree-elevated net of the same carrier to the
/// marcher, and admitted a two-row rational net whose weights differ along its
/// rulings — a curve off the plane that nothing downstream catches. Escape
/// hatch for measurement: `BREP_ISO_GATE_GEOMETRY=0` runs the count gate, and
/// with `BREP_FIT_CENSUS=1` compares every pair that then marches against the
/// exact curve.
pub(super) fn iso_gate() -> IsoGate {
    if std::env::var("BREP_ISO_GATE_GEOMETRY").as_deref() == Ok("0") {
        IsoGate::Count
    } else {
        IsoGate::Geometry
    }
}

/// How far one direction of a net is from the rulings the iso lane assumes.
#[derive(Clone, Copy, Debug)]
pub(super) struct RulingMeasure {
    /// Every line of control points across the direction passes
    /// [`NurbsCurve::straight_segment`] at the lane's tolerance.
    pub(super) straight: bool,
    /// Largest distance of a control point from where a constant-speed
    /// parameterization puts it: `P_0 + (ξ_k − ξ_0)·(P_n − P_0)/(ξ_n − ξ_0)`
    /// at its Greville abscissa `ξ_k`.
    pub(super) affine_gap: f64,
    /// Largest `|w_k / w_0 − 1|·|P_n − P_0|` along a line: the displacement a
    /// weight spread can put on the ruling's parameterization.
    pub(super) weight_gap: f64,
}

impl RulingMeasure {
    pub(super) fn admits(&self, tolerance: f64) -> bool {
        self.straight && self.affine_gap <= tolerance && self.weight_gap <= tolerance
    }
}

/// Measure whether every ruling of `ruled` across one direction is a straight
/// segment traced at constant speed — the property the iso lane's root
/// computation consumes.
///
/// The lane finds where a ruling crosses the plane from the plane distances
/// of the ruling's two ENDS, as the linear fraction `d₀/(d₀ − d₁)` of the
/// parameter domain, and then takes the iso-curve at that parameter. That is
/// exact when the ruling is straight (so one crossing, located by its ends)
/// AND its parameterization is affine (so the crossing's parameter is that
/// fraction). Straightness alone is the image; the fraction also reads the
/// parameter.
///
/// Both are decided on the net, at any degree and any row count. Weights are
/// strictly positive, so every point of a line is a convex combination of its
/// control points: collinear control points make it straight
/// ([`NurbsCurve::straight_segment`]). B-splines reproduce linear functions
/// from their Greville abscissae (`Σ N_k(t)·ξ_k = t`), so control points at
/// `A + B·ξ_k` with equal weights make it `A + B·t` exactly — and because a
/// row's weights are shared across the whole row, every ruling blended
/// between rows inherits it, within the same distance. A two-row degree-1
/// polynomial net passes both by construction; a knot insertion, a degree
/// elevation or a rejoin at the right parameter keeps passing; a rejoin
/// whose interior pole is not at its knot's fraction, or a rational line
/// whose weights differ along it, does not, whatever it counts.
pub(super) fn rulings_are_straight_and_affine(
    ruled: &NurbsSurface,
    linear_v: bool,
    tolerance: f64,
) -> RulingMeasure {
    let declined = RulingMeasure {
        straight: false,
        affine_gap: f64::INFINITY,
        weight_gap: f64::INFINITY,
    };
    let rows = &ruled.control_points;
    let (degree, knots, lines): (usize, &Vec<f64>, Vec<Vec<Vec4>>) = if linear_v {
        (ruled.degree_v, &ruled.knots_v, rows.clone())
    } else {
        (
            ruled.degree_u,
            &ruled.knots_u,
            (0..rows.first().map(|row| row.len()).unwrap_or(0))
                .map(|column| rows.iter().map(|row| row[column]).collect())
                .collect(),
        )
    };
    if degree == 0 || lines.is_empty() {
        return declined;
    }
    let mut measure = RulingMeasure {
        straight: true,
        affine_gap: 0.0,
        weight_gap: 0.0,
    };
    for line in lines {
        let count = line.len();
        if count < 2 || knots.len() != count + degree + 1 {
            return declined;
        }
        let greville = (0..count)
            .map(|index| knots[index + 1..=index + degree].iter().sum::<f64>() / degree as f64)
            .collect::<Vec<_>>();
        let span = greville[count - 1] - greville[0];
        let Ok(points) = line
            .iter()
            .map(|control| control.point())
            .collect::<Result<Vec<Vec3>, _>>()
        else {
            return declined;
        };
        if span <= 0.0 || line[0].w <= 0.0 {
            return declined;
        }
        let chord = points[count - 1].sub(points[0]);
        for index in 0..count {
            let expected = points[0].add(chord.scale((greville[index] - greville[0]) / span));
            measure.affine_gap = measure.affine_gap.max(points[index].sub(expected).length());
            measure.weight_gap = measure
                .weight_gap
                .max((line[index].w / line[0].w - 1.0).abs() * chord.length());
        }
        match NurbsCurve::new(degree, knots.clone(), line) {
            Ok(curve) => measure.straight &= curve.straight_segment(tolerance).is_some(),
            Err(_) => return declined,
        }
    }
    measure
}

/// How [`planar_linear_iso_intersection`] answered one direction.
#[derive(Clone, Debug)]
pub(super) enum IsoOutcome {
    /// The gate declined the direction.
    Gate,
    /// A ruling runs parallel to the plane, or crosses it outside its domain.
    Root,
    /// The rulings cross at different parameters: not an iso-curve.
    RootAgreement,
    /// The iso-curve failed `curve_lies_on_surface` against the plane.
    OffPlane,
    /// The iso-curve collapses to a point: every ruling meets the plane at a
    /// shared pole (a cone whose apex lies in the plane). That point is not
    /// the section — a plane through a cone's apex cuts it along rulings —
    /// so the lane declines and the analytic/marched lanes answer.
    Pole,
    Curve(NurbsCurve),
}

impl IsoOutcome {
    pub(super) fn label(&self) -> &'static str {
        match self {
            IsoOutcome::Gate => "gate",
            IsoOutcome::Root => "root",
            IsoOutcome::RootAgreement => "root_agreement",
            IsoOutcome::OffPlane => "off_plane",
            IsoOutcome::Pole => "pole",
            IsoOutcome::Curve(_) => "curve",
        }
    }
}

/// Recover an exact isoparametric intersection when an affine plane slices
/// a surface that is linear across one parameter direction. General SSI can
/// miss this case when the other direction is a high-degree rational curve:
/// all seeds lie on two distant trim boundaries even though the intersection
/// itself is a simple interior iso-curve.
///
/// "Linear across one direction" is decided by [`iso_gate`] — by default on the
/// net's geometry ([`rulings_are_straight_and_affine`]), at any degree and any
/// row count — and the root, root-agreement and `curve_lies_on_surface` checks
/// below stand behind it unchanged.
fn planar_linear_iso_intersection(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    for linear_v in [true, false] {
        if let IsoOutcome::Curve(curve) =
            planar_linear_iso_direction(plane, ruled, tolerance, linear_v, iso_gate())?
        {
            return Ok(Some(curve));
        }
    }
    Ok(None)
}

/// One direction of [`planar_linear_iso_intersection`] under a named gate.
pub(super) fn planar_linear_iso_direction(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    tolerance: f64,
    linear_v: bool,
    gate: IsoGate,
) -> Result<IsoOutcome, KernelRefusal> {
    if !plane
        .is_affine()
        .or_refuse(KernelStage::Intersect, "is_affine")?
    {
        return Ok(IsoOutcome::Gate);
    }
    let admitted = match gate {
        IsoGate::Count => {
            if linear_v {
                ruled.degree_v == 1 && ruled.control_points.iter().all(|row| row.len() == 2)
            } else {
                ruled.degree_u == 1 && ruled.control_points.len() == 2
            }
        }
        IsoGate::Geometry => rulings_are_straight_and_affine(ruled, linear_v, tolerance).admits(tolerance),
    };
    if !admitted {
        return Ok(IsoOutcome::Gate);
    }
    let plane_u = KnotVector::new(plane.knots_u.clone(), plane.degree_u)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let plane_v = KnotVector::new(plane.knots_v.clone(), plane.degree_v)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let derivatives = plane
        .derivatives(plane_u, plane_v, 1)
        .or_refuse(KernelStage::Intersect, "derivatives")?;
    let origin = derivatives[0][0];
    let normal = derivatives[1][0]
        .cross(derivatives[0][1])
        .normalized()
        .or_refuse(KernelStage::Intersect, "normalized")?;
    let signed_distance = |point: Vec3| point.sub(origin).dot(normal);
    let root_tolerance = (tolerance * 100.0).max(1e-7);

    let u_domain = KnotVector::new(ruled.knots_u.clone(), ruled.degree_u)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain();
    let v_domain = KnotVector::new(ruled.knots_v.clone(), ruled.degree_v)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain();
    let mut roots = Vec::new();
    for fraction in [0.0, 0.2, 0.5, 0.8, 1.0] {
        let (first, second) = if linear_v {
            let u = u_domain[0] + (u_domain[1] - u_domain[0]) * fraction;
            (
                ruled
                    .evaluate(u, v_domain[0])
                    .or_refuse(KernelStage::Intersect, "evaluate")?,
                ruled
                    .evaluate(u, v_domain[1])
                    .or_refuse(KernelStage::Intersect, "evaluate")?,
            )
        } else {
            let v = v_domain[0] + (v_domain[1] - v_domain[0]) * fraction;
            (
                ruled
                    .evaluate(u_domain[0], v)
                    .or_refuse(KernelStage::Intersect, "evaluate")?,
                ruled
                    .evaluate(u_domain[1], v)
                    .or_refuse(KernelStage::Intersect, "evaluate")?,
            )
        };
        let first_distance = signed_distance(first);
        let second_distance = signed_distance(second);
        let denominator = first_distance - second_distance;
        if denominator.abs() <= root_tolerance {
            return Ok(IsoOutcome::Root);
        }
        let root = first_distance / denominator;
        if root < -root_tolerance || root > 1.0 + root_tolerance {
            return Ok(IsoOutcome::Root);
        }
        roots.push(root.clamp(0.0, 1.0));
    }
    let root = roots.iter().sum::<f64>() / roots.len() as f64;
    if roots
        .iter()
        .any(|candidate| (candidate - root).abs() > root_tolerance * 10.0)
    {
        return Ok(IsoOutcome::RootAgreement);
    }
    let curve = if linear_v {
        ruled
            .iso_curve_v(v_domain[0] + (v_domain[1] - v_domain[0]) * root)
            .or_refuse(KernelStage::Intersect, "iso_curve_v")?
    } else {
        ruled
            .iso_curve_u(u_domain[0] + (u_domain[1] - u_domain[0]) * root)
            .or_refuse(KernelStage::Intersect, "iso_curve_u")?
    };
    let anchor = curve
        .control_points
        .first()
        .and_then(|control| control.point().ok());
    if let Some(anchor) = anchor {
        let collapsed = curve.control_points.iter().all(|control| {
            control
                .point()
                .map(|point| point.sub(anchor).length() <= tolerance)
                .unwrap_or(false)
        });
        if collapsed {
            return Ok(IsoOutcome::Pole);
        }
    }
    if !curve_lies_on_surface(&curve, plane, tolerance)? {
        return Ok(IsoOutcome::OffPlane);
    }
    Ok(IsoOutcome::Curve(curve))
}

/// [`planar_iso_intersection`] under a named gate — what the census asks of
/// the gate the boolean does NOT run.
pub(super) fn planar_iso_intersection_with(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    gate: IsoGate,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    for (plane, ruled) in [(first, second), (second, first)] {
        for linear_v in [true, false] {
            if let IsoOutcome::Curve(curve) =
                planar_linear_iso_direction(plane, ruled, tolerance, linear_v, gate)?
            {
                return Ok(Some(curve));
            }
        }
    }
    Ok(None)
}

/// CENSUS ONLY (`BREP_FIT_CENSUS=1`, observation, no behaviour change): one
/// line per direction on which the two iso gates answer differently, with the
/// geometry each was shown and the check that declined each.
pub(super) fn report_iso_gate_census(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    for (plane, ruled) in [(first, second), (second, first)] {
        if !plane
            .is_affine()
            .or_refuse(KernelStage::Intersect, "is_affine")?
        {
            continue;
        }
        for linear_v in [true, false] {
            let count = planar_linear_iso_direction(plane, ruled, tolerance, linear_v, IsoGate::Count)?;
            let geometry =
                planar_linear_iso_direction(plane, ruled, tolerance, linear_v, IsoGate::Geometry)?;
            if matches!((&count, &geometry), (IsoOutcome::Gate, IsoOutcome::Gate)) {
                continue;
            }
            let same = match (&count, &geometry) {
                (IsoOutcome::Curve(a), IsoOutcome::Curve(b)) => curves_bitwise_equal(a, b),
                (a, b) => a.label() == b.label(),
            };
            if same {
                continue;
            }
            let measure = rulings_are_straight_and_affine(ruled, linear_v, tolerance);
            let (rows, columns) = (
                ruled.control_points.len(),
                ruled.control_points.first().map(|row| row.len()).unwrap_or(0),
            );
            eprintln!(
                "FITCENSUS iso_gate linear_v={linear_v} degree={} net={rows}x{columns} count={} \
                 collinearity={:.3e} straight={} affine_gap={:.3e} weight_gap={:.3e} \
                 count_gate={} geometry_gate={} thread={}",
                if linear_v { ruled.degree_v } else { ruled.degree_u },
                if linear_v { columns } else { rows },
                control_collinearity(ruled, linear_v),
                measure.straight,
                measure.affine_gap,
                measure.weight_gap,
                count.label(),
                geometry.label(),
                std::thread::current().name().unwrap_or("unnamed").replace(' ', "_"),
            );
        }
    }
    Ok(())
}

/// The same curve to the bit: degree, knots and every homogeneous coordinate.
pub(super) fn curves_bitwise_equal(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && a.knots.len() == b.knots.len()
        && a.knots.iter().zip(&b.knots).all(|(x, y)| x.to_bits() == y.to_bits())
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(p, q)| {
            [p.x, p.y, p.z, p.w]
                .iter()
                .zip([q.x, q.y, q.z, q.w])
                .all(|(x, y)| x.to_bits() == y.to_bits())
        })
}

/// Largest distance of an interior control point from the straight line through
/// its row's (or column's) ends — how far a direction is from straight, read
/// off the control points alone. Census helper; see
/// [`report_iso_gate_census`].
fn control_collinearity(ruled: &NurbsSurface, linear_v: bool) -> f64 {
    let rows = &ruled.control_points;
    let mut worst = 0.0_f64;
    let lines: Vec<Vec<Vec3>> = if linear_v {
        rows.iter()
            .map(|row| {
                row.iter()
                    .map(|point| point.point().unwrap_or_default())
                    .collect()
            })
            .collect()
    } else {
        (0..rows.first().map(|row| row.len()).unwrap_or(0))
            .map(|column| {
                rows.iter()
                    .map(|row| row[column].point().unwrap_or_default())
                    .collect()
            })
            .collect()
    };
    for line in lines {
        if line.len() < 3 {
            continue;
        }
        let (first, last) = (line[0], line[line.len() - 1]);
        let direction = last.sub(first);
        let length_squared = direction.length_squared();
        for point in &line[1..line.len() - 1] {
            let distance = if length_squared <= 0.0 {
                point.sub(first).length()
            } else {
                let fraction =
                    (point.sub(first).dot(direction) / length_squared).clamp(0.0, 1.0);
                point.sub(first.add(direction.scale(fraction))).length()
            };
            worst = worst.max(distance);
        }
    }
    worst
}

pub(super) fn planar_iso_intersection(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    if let Some(curve) = planar_linear_iso_intersection(first, second, tolerance)? {
        return Ok(Some(curve));
    }
    planar_linear_iso_intersection(second, first, tolerance)
}

fn coplanar_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    if first.degree_u != 1 || first.degree_v != 1 || second.degree_u != 1 || second.degree_v != 1 {
        return Ok(false);
    }
    let first_u = KnotVector::new(first.knots_u.clone(), first.degree_u)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let first_v = KnotVector::new(first.knots_v.clone(), first.degree_v)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let second_u = KnotVector::new(second.knots_u.clone(), second.degree_u)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let second_v = KnotVector::new(second.knots_v.clone(), second.degree_v)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain()[0];
    let a = first
        .derivatives(first_u, first_v, 1)
        .or_refuse(KernelStage::Intersect, "derivatives")?;
    let b = second
        .derivatives(second_u, second_v, 1)
        .or_refuse(KernelStage::Intersect, "derivatives")?;
    let normal_a = a[1][0]
        .cross(a[0][1])
        .normalized()
        .or_refuse(KernelStage::Intersect, "normalized")?;
    let normal_b = b[1][0]
        .cross(b[0][1])
        .normalized()
        .or_refuse(KernelStage::Intersect, "normalized")?;
    Ok(normal_a.dot(normal_b).abs() >= 1.0 - 1e-9
        && b[0][0].sub(a[0][0]).dot(normal_a).abs()
            <= (tolerance * 10.0).max(COINCIDENCE_DISTANCE_FLOOR))
}

pub(super) fn cosurface_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let first_planar = first.degree_u == 1 && first.degree_v == 1;
    let second_planar = second.degree_u == 1 && second.degree_v == 1;
    if first_planar && second_planar {
        return coplanar_pair(first, second, tolerance);
    }
    if first_planar != second_planar {
        return Ok(false);
    }
    let bounds_a = Aabb::from_surface_controls(first)
        .or_refuse(KernelStage::Intersect, "from_surface_controls")?;
    let bounds_b = Aabb::from_surface_controls(second)
        .or_refuse(KernelStage::Intersect, "from_surface_controls")?;
    if !bounds_a.intersects(bounds_b, tolerance * 10.0) {
        return Ok(false);
    }
    let scale = bounds_a.diagonal().max(bounds_b.diagonal());
    let probe = bounds_b.expanded(scale * 0.01);
    let u_domain = KnotVector::new(first.knots_u.clone(), first.degree_u)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain();
    let v_domain = KnotVector::new(first.knots_v.clone(), first.degree_v)
        .or_refuse(KernelStage::Intersect, "new")?
        .domain();
    let mut samples = Vec::new();
    for fu in [0.13, 0.37, 0.61, 0.89] {
        for fv in [0.17, 0.43, 0.71, 0.93] {
            let u = u_domain[0] + (u_domain[1] - u_domain[0]) * fu;
            let v = v_domain[0] + (v_domain[1] - v_domain[0]) * fv;
            let point = first
                .evaluate(u, v)
                .or_refuse(KernelStage::Intersect, "evaluate")?;
            if probe.contains(point) {
                samples.push((point, u, v));
            }
        }
    }
    if samples.len() < 3 {
        return Ok(false);
    }
    for (point, u, v) in samples {
        let projection = project_point_to_surface(second, point)
            .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
        if projection.distance > (tolerance * 100.0).max(COINCIDENCE_DISTANCE_FLOOR) {
            return Ok(false);
        }
        if let (Ok(normal_a), Ok(normal_b)) = (
            first.normal(u, v),
            second.normal(projection.u, projection.v),
        ) {
            if normal_a.dot(normal_b).abs() < 1.0 - 1e-6 {
                return Ok(false);
            }
        }
    }
    Ok(true)
}
