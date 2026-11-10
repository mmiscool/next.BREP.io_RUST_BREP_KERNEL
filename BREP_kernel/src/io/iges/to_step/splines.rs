//! Surfaces and curves STEP has no entity for, said exactly as splines: the
//! 114 parametric spline surface (bicubic patches in power form), the 118
//! ruled surface, and the 130 offset curve of a line or an arc.
//!
//! Each is the same geometry, not a fit: a power-form patch is one Bezier
//! patch, a ruling between two curves that share a parameter is a degree 1
//! blend of their control nets, and an offset line or arc is a line or an arc.
//! A case with no exact spline form is refused by name.

use super::{EmittedCurve, EmittedSurface, Chart, Transform, Translator};
use super::super::reader::ParsedEntity;
use crate::coons::compat::make_family_compatible;
use crate::{make_arc, make_line, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// How a curve's IGES parameter runs, which decides whether two curves can
/// share a ruling exactly.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Pace {
    /// Proportional to arc length and to the spline's own parameter (a line).
    Even,
    /// The spline's own parameter (a 112 or 126), not arc length.
    Spline,
    /// The angle of an arc with this sweep, in radians.
    Angle(f64),
}

/// Bezier control coefficients of one cubic in power form: `B_k = Σ_p M[k][p] a_p h^p`.
const POWER_TO_BEZIER: [[f64; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 1.0 / 3.0, 0.0, 0.0],
    [1.0, 2.0 / 3.0, 1.0 / 3.0, 0.0],
    [1.0, 1.0, 1.0, 1.0],
];

impl Translator<'_> {
    /// A 114: patch (i, j) holds X, Y and Z as `Σ a_pq s^p t^q` (16 each, `p`
    /// fastest) in `s = u - TU(i)`, `t = v - TV(j)`. The sets run with `j`
    /// fastest, each `i` followed by one unused set for `j = N`, and an unused
    /// row for `i = M` closes the list (a writer may leave those out). Each
    /// patch is one Bezier patch of a degree 3 spline on the same breaks.
    pub(super) fn parametric_spline_surface(
        &mut self,
        entity: &ParsedEntity,
        transform: &Transform,
    ) -> Result<EmittedSurface, String> {
        let (m, n) = (entity.int(2)?, entity.int(3)?);
        if m < 1 || n < 1 {
            return Err(format!("114 has {m} by {n} patches"));
        }
        let (m, n) = (m as usize, n as usize);
        let read = |from: usize, count: usize| -> Result<Vec<f64>, String> {
            (from..from + count).map(|index| entity.real(index)).collect()
        };
        let breaks_u = read(4, m + 1)?;
        let breaks_v = read(5 + m, n + 1)?;
        if breaks_u.windows(2).chain(breaks_v.windows(2)).any(|w| w[1] <= w[0]) {
            return Err("114 breakpoints do not increase".into());
        }
        let first = 6 + m + n;
        // With the unused sets each row of patches is N + 1 sets long.
        let stride = if entity.real(first + 48 * ((m + 1) * (n + 1)) - 1).is_ok() { n + 1 } else { n };
        let rows = 3 * m + 1;
        let columns = 3 * n + 1;
        let mut grid: Vec<Vec<Option<Vec3>>> = vec![vec![None; columns]; rows];
        for i in 0..m {
            let hu = breaks_u[i + 1] - breaks_u[i];
            for j in 0..n {
                let hv = breaks_v[j + 1] - breaks_v[j];
                let base = first + 48 * (i * stride + j);
                let mut power = [[[0.0f64; 3]; 4]; 4];
                for axis in 0..3 {
                    for q in 0..4 {
                        for p in 0..4 {
                            let a = entity.real(base + axis * 16 + p + 4 * q)?;
                            power[p][q][axis] = a * hu.powi(p as i32) * hv.powi(q as i32);
                        }
                    }
                }
                for k in 0..4 {
                    for l in 0..4 {
                        let mut point = [0.0f64; 3];
                        for p in 0..=k {
                            for q in 0..=l {
                                let factor = POWER_TO_BEZIER[k][p] * POWER_TO_BEZIER[l][q];
                                for axis in 0..3 {
                                    point[axis] += factor * power[p][q][axis];
                                }
                            }
                        }
                        let point = self.location(transform, point);
                        let slot = &mut grid[3 * i + k][3 * j + l];
                        match slot {
                            Some(have) if have.sub(point).length() > self.resolution => {
                                return Err(format!(
                                    "114 patches ({i}, {j}) and their neighbour do not meet: {:.3e} mm apart",
                                    have.sub(point).length()
                                ));
                            }
                            Some(_) => {}
                            None => *slot = Some(point),
                        }
                    }
                }
            }
        }
        let control = grid
            .into_iter()
            .map(|row| row.into_iter().map(|p| Vec4::from_point(p.expect("every slot is a patch corner"), 1.0)).collect())
            .collect();
        let surface = NurbsSurface::new(3, 3, cubic_knots(&breaks_u), cubic_knots(&breaks_v), control)?;
        Ok(self.spline_surface(&surface))
    }

    /// A 118: the rulings join `C1` and `C2` at equal fractions of their
    /// parameters (form 1) or of their arc lengths (form 0), `C2` read
    /// backwards when DIRFLG is 1. That is a degree 1 blend of the two control
    /// nets exactly when both curves share one parameter function: two
    /// polynomial curves under form 1, two lines, or two arcs of one sweep.
    pub(super) fn ruled(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let arc_length = entity.de.form == 0;
        let (first, first_pace) = self.spline_of(entity.int(0)?, transform)?;
        let (second, second_pace) = self.spline_of(entity.int(1)?, transform)?;
        let second = if entity.int(2)? == 1 { second.reversed()? } else { second };
        let shared = match (first_pace, second_pace) {
            (Pace::Even, Pace::Even) => true,
            (Pace::Even | Pace::Spline, Pace::Even | Pace::Spline) => !arc_length,
            (Pace::Angle(a), Pace::Angle(b)) => (a - b).abs() <= 1e-12 * a.abs().max(b.abs()),
            _ => false,
        };
        if !shared {
            return Err(format!(
                "a ruled surface (118 form {}) between {} and {} has no exact spline form",
                entity.de.form,
                pace_name(first_pace),
                pace_name(second_pace)
            ));
        }
        let (rails, _) = make_family_compatible(&[first, second], 1)?;
        let (one, two) = (&rails[0], &rails[1]);
        // A degree 1 blend of rational rails is the ruling only when both share
        // one denominator.
        let same_weights = one
            .control_points
            .iter()
            .zip(&two.control_points)
            .all(|(a, b)| (a.w - b.w).abs() <= 1e-12 * a.w.abs().max(1.0));
        if !same_weights {
            return Err("a ruled surface (118) between rational curves of different weights has no exact spline form".into());
        }
        let control = one.control_points.iter().zip(&two.control_points).map(|(a, b)| vec![*a, *b]).collect();
        let surface = NurbsSurface::new(one.degree, 1, one.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], control)?;
        Ok(self.spline_surface(&surface))
    }

    /// A curve as a spline in model millimetres, with how its IGES parameter
    /// runs. Only curves whose IGES parameter the spline keeps.
    fn spline_of(&self, pointer: i64, parent: &Transform) -> Result<(NurbsCurve, Pace), String> {
        let entity = self.file.entity(pointer)?;
        let transform = self.placed(entity, parent)?;
        let moved = |curve: NurbsCurve| -> Result<NurbsCurve, String> {
            let points = curve
                .control_points
                .iter()
                .map(|h| {
                    let p = h.point()?;
                    Ok(Vec4::from_point(self.location(&transform, [p.x, p.y, p.z]), h.w))
                })
                .collect::<Result<Vec<_>, String>>()?;
            NurbsCurve::new(curve.degree, curve.knots.clone(), points)
        };
        match entity.de.entity_type {
            110 => {
                let a = Vec3::new(entity.real(0)?, entity.real(1)?, entity.real(2)?);
                let b = Vec3::new(entity.real(3)?, entity.real(4)?, entity.real(5)?);
                Ok((moved(make_line(a, b)?)?, Pace::Even))
            }
            100 => {
                let z = entity.real(0)?;
                let (cx, cy) = (entity.real(1)?, entity.real(2)?);
                let (sx, sy) = (entity.real(3)?, entity.real(4)?);
                let (ex, ey) = (entity.real(5)?, entity.real(6)?);
                let radius = (sx - cx).hypot(sy - cy);
                let from = (sy - cy).atan2(sx - cx);
                let mut to = (ey - cy).atan2(ex - cx);
                if to <= from + 1e-12 {
                    to += std::f64::consts::TAU;
                }
                let arc = make_arc(Vec3::new(cx, cy, z), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), radius, from, to)?;
                Ok((moved(arc)?, Pace::Angle(to - from)))
            }
            126 => {
                let count = entity.int(0)? as usize + 1;
                let degree = entity.int(1)? as usize;
                let knots: Vec<f64> = (6..6 + count + degree + 1).map(|i| entity.real(i)).collect::<Result<_, _>>()?;
                let base = 6 + knots.len();
                let mut points = Vec::with_capacity(count);
                for i in 0..count {
                    let w = entity.real(base + i)?;
                    let at = base + count + 3 * i;
                    let p = Vec3::new(entity.real(at)?, entity.real(at + 1)?, entity.real(at + 2)?);
                    points.push(Vec4::from_point(p, w));
                }
                let tail = base + 4 * count;
                let (v0, v1) = (entity.real(tail)?, entity.real(tail + 1)?);
                let mut curve = NurbsCurve::new(degree, knots, points)?;
                // The curve is the part between V(0) and V(1).
                let [d0, d1] = curve.domain()?;
                let slack = 1e-12 * (d1 - d0).abs().max(1.0);
                if v1 < d1 - slack && v1 > d0 + slack {
                    curve = curve.split(v1)?.0;
                }
                if v0 > d0 + slack && v0 < d1 - slack {
                    curve = curve.split(v0)?.1;
                }
                let pace = if degree == 1 && count == 2 { Pace::Even } else { Pace::Spline };
                Ok((moved(curve)?, pace))
            }
            112 => {
                let (control, multiplicities, knots) = power_spline(entity)?;
                let mut full = Vec::new();
                for (knot, &times) in knots.iter().zip(&multiplicities) {
                    full.extend(std::iter::repeat_n(*knot, times));
                }
                let points = control.into_iter().map(|p| Vec4::from_point(Vec3::new(p[0], p[1], p[2]), 1.0)).collect();
                Ok((moved(NurbsCurve::new(3, full, points)?)?, Pace::Spline))
            }
            other => Err(format!("curve DE {pointer} (entity {other}) cannot be a rail of a ruled surface here")),
        }
    }

    /// A 130 of a line or an arc in the plane the 130 names: the offset of a
    /// line is the line moved by `d (N × T)` and the offset of an arc the
    /// concentric arc. OpenCASCADE reads and writes the offset this way (it
    /// reverses the stated normal both ways, after a real file, CTS22023), so
    /// files from it and from what it reads agree. Only a uniform distance.
    pub(super) fn offset_curve(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        match self.offset_geometry(entity, transform)? {
            Offset::Line { start, end } => {
                let origin = self.point_id(start);
                let along = self.direction_id(end.sub(start).normalized()?);
                let vector = self.add(format!("VECTOR('',#{along},1.)"));
                let id = self.add(format!("LINE('',#{origin},#{vector})"));
                Ok(EmittedCurve { id, start, end, collapsed: false })
            }
            Offset::Arc { center, axis, radius, x, sweep } => {
                let placement = self.placement_id(center, axis, x);
                let id = self.add(format!("CIRCLE('',#{placement},{})", super::real(radius)));
                let start = center.add(x.scale(radius));
                let end = Offset::arc_point(center, axis, x, radius, sweep);
                Ok(EmittedCurve { id, start, end, collapsed: false })
            }
        }
    }

    /// A point at fraction `t` along a 130, as `offset_curve` emits it.
    pub(super) fn offset_point(&self, entity: &ParsedEntity, transform: &Transform, t: f64) -> Option<Vec3> {
        match self.offset_geometry(entity, transform).ok()? {
            Offset::Line { start, end } => Some(start.add(end.sub(start).scale(t))),
            Offset::Arc { center, axis, radius, x, sweep } => Some(Offset::arc_point(center, axis, x, radius, sweep * t)),
        }
    }

    /// The exact curve a 130 of a line or an arc is, in model millimetres.
    /// Fields: BC, FLAG, BDDC, C, TT (taper type), D1, TD1, D2, TD2, VX, VY,
    /// VZ, TT1, TT2; TT1 and TT2 bound the part of the base that is offset.
    /// The standard states them in the base's IGES parameter, OpenCASCADE in
    /// its own (arc length on a line), so only the whole base, where the two
    /// readings agree, is read; part of a base is refused by name.
    pub(super) fn offset_geometry(&self, entity: &ParsedEntity, transform: &Transform) -> Result<Offset, String> {
        let (base_pointer, flag) = (entity.int(0)?, entity.int(1)?);
        if flag != 1 {
            return Err(format!("an offset curve (130) whose distance varies (flag {flag}) has no exact form here"));
        }
        let distance = entity.real(5)?;
        let normal_local = [entity.real(9)?, entity.real(10)?, entity.real(11)?];
        let (from, to) = (entity.real(12)?, entity.real(13)?);
        let base = self.file.entity(base_pointer)?;
        let base_transform = self.placed(base, transform)?;
        let uniform = transform.uniform_scale().ok_or("the 124 on this offset curve is not a rigid motion with a uniform scale")?;
        let normal = super::vec3(transform.vector(normal_local)).normalized()?;
        let distance = distance * uniform * self.scale;
        match base.de.entity_type {
            110 => {
                let local = |i: usize| -> Result<[f64; 3], String> { Ok([base.real(i)?, base.real(i + 1)?, base.real(i + 2)?]) };
                let (first, last) = (local(0)?, local(3)?);
                let length = ((last[0] - first[0]).powi(2) + (last[1] - first[1]).powi(2) + (last[2] - first[2]).powi(2)).sqrt();
                // The whole line is 0 to 1 by the standard, 0 to its length by OpenCASCADE.
                let near = |a: f64, b: f64| (a - b).abs() <= 1e-9 * length.max(1.0);
                if !(near(from, 0.0) && (near(to, 1.0) || near(to, length))) {
                    return Err(format!(
                        "an offset curve (130) over {from} to {to} of part of a line has no single reading (the standard and OpenCASCADE place it differently)"
                    ));
                }
                let a = self.location(&base_transform, first);
                let b = self.location(&base_transform, last);
                let tangent = b.sub(a).normalized()?;
                let shift = normal.cross(tangent).normalized()?.scale(distance);
                Ok(Offset::Line { start: a.add(shift), end: b.add(shift) })
            }
            100 => {
                let z = base.real(0)?;
                let (cx, cy) = (base.real(1)?, base.real(2)?);
                let (sx, sy, ex, ey) = (base.real(3)?, base.real(4)?, base.real(5)?, base.real(6)?);
                let begin = (sy - cy).atan2(sx - cx);
                let mut finish = (ey - cy).atan2(ex - cx);
                if finish <= begin + 1e-12 {
                    finish += std::f64::consts::TAU;
                }
                let sweep = finish - begin;
                // OpenCASCADE writes and reads the range as the arc's angles
                // about the IGES x axis; that is the one reading of the whole
                // arc taken. Any other range is refused by name.
                let near = |a: f64, b: f64| (a - b).abs() <= 1e-9 * std::f64::consts::TAU;
                let turns = ((from - begin) / std::f64::consts::TAU).round();
                let whole = near(from - turns * std::f64::consts::TAU, begin) && near(to - from, sweep);
                if !whole {
                    return Err(format!("an offset curve (130) over {from} to {to} of part of an arc has no single reading (the standard and OpenCASCADE place it differently)"));
                }
                let center = self.location(&base_transform, [cx, cy, z]);
                let start = self.location(&base_transform, [sx, sy, z]);
                let axis = super::vec3(base_transform.vector([0.0, 0.0, 1.0])).normalized()?;
                if axis.cross(normal).length() > 1e-9 {
                    return Err("an offset curve (130) of an arc out of the arc's plane has no exact form".into());
                }
                let radial = start.sub(center);
                let radius = radial.length();
                // A mirroring 124 turns the arc the other way about the axis.
                let axis = if base_transform.reflects() { axis.scale(-1.0) } else { axis };
                let tangent = axis.cross(radial).normalized()?;
                // N × T against the outward radial: outward moves the arc out.
                let outward = normal.cross(tangent).dot(radial) > 0.0;
                let radius_new = if outward { radius + distance } else { radius - distance };
                if radius_new <= self.resolution {
                    return Err("an offset curve (130) shrinks its arc to nothing".into());
                }
                Ok(Offset::Arc { center, axis, radius: radius_new, x: radial.normalized()?, sweep })
            }
            other => Err(format!("an offset curve (130) of entity {other} has no exact form here")),
        }
    }

    /// A spline surface as Part 21, rows of constant u as STEP lists them.
    fn spline_surface(&mut self, surface: &NurbsSurface) -> EmittedSurface {
        let rational = surface.control_points.iter().flatten().any(|h| (h.w - 1.0).abs() > 1e-12);
        let mut rows = Vec::with_capacity(surface.control_points.len());
        let mut weight_rows = Vec::with_capacity(surface.control_points.len());
        for row in &surface.control_points {
            let ids: Vec<String> = row
                .iter()
                .map(|h| {
                    let p = Vec3::new(h.x / h.w, h.y / h.w, h.z / h.w);
                    format!("#{}", self.point_id(p))
                })
                .collect();
            rows.push(format!("({})", ids.join(",")));
            weight_rows.push(format!("({})", super::join_real(&row.iter().map(|h| h.w).collect::<Vec<_>>())));
        }
        let rows = rows.join(",");
        let (degree_u, degree_v) = (surface.degree_u, surface.degree_v);
        let (distinct_u, mults_u) = super::knot_runs(&surface.knots_u);
        let (distinct_v, mults_v) = super::knot_runs(&surface.knots_v);
        let (mults_u, mults_v) = (super::join_usize(&mults_u), super::join_usize(&mults_v));
        let (values_u, values_v) = (super::join_real(&distinct_u), super::join_real(&distinct_v));
        let id = if rational {
            self.add(format!(
                "(BOUNDED_SURFACE()B_SPLINE_SURFACE({degree_u},{degree_v},({rows}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS(({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({}))REPRESENTATION_ITEM('')SURFACE())",
                weight_rows.join(",")
            ))
        } else {
            self.add(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{degree_u},{degree_v},({rows}),.UNSPECIFIED.,.F.,.F.,.F.,({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)"
            ))
        };
        let poles = self.spline_poles(surface);
        let spline = Some(std::rc::Rc::new(surface.clone()));
        EmittedSurface { id, reversed: false, seam: None, poles, chart: Some(Chart::Parameter), domain_v: None, interior: None, spline }
    }

    /// The points where a spline surface's boundaries collapse, as
    /// `boundary_poles` finds them on a 128.
    fn spline_poles(&self, surface: &NurbsSurface) -> Option<Vec<Vec3>> {
        const STATIONS: usize = 16;
        let [u0, u1] = surface.domain_u().ok()?;
        let [v0, v1] = surface.domain_v().ok()?;
        let mut poles = Vec::new();
        let mut boundary = |point_at: &dyn Fn(f64) -> (f64, f64)| -> Option<()> {
            let mut points = Vec::with_capacity(STATIONS + 1);
            for i in 0..=STATIONS {
                let (u, v) = point_at(i as f64 / STATIONS as f64);
                points.push(surface.evaluate(u, v).ok()?);
            }
            if points.iter().all(|p| p.sub(points[0]).length() <= self.resolution) {
                poles.push(points[0]);
            }
            Some(())
        };
        boundary(&|t| (u0, v0 + (v1 - v0) * t))?;
        boundary(&|t| (u1, v0 + (v1 - v0) * t))?;
        boundary(&|t| (u0 + (u1 - u0) * t, v0))?;
        boundary(&|t| (u0 + (u1 - u0) * t, v1))?;
        Some(poles)
    }
}

/// A 112's cubic segments as Bezier spans in the 112's own frame: control
/// points, knot multiplicities and the distinct knots (its breakpoints).
pub(super) fn power_spline(entity: &ParsedEntity) -> Result<(Vec<[f64; 3]>, Vec<usize>, Vec<f64>), String> {
    let segments = entity.int(3)? as usize;
    if segments == 0 {
        return Err("112 has no segments".into());
    }
    let breaks = 4;
    let knots: Vec<f64> = (0..=segments).map(|index| entity.real(breaks + index)).collect::<Result<_, _>>()?;
    let coefficients = breaks + segments + 1;
    let mut control = Vec::with_capacity(3 * segments + 1);
    for segment in 0..segments {
        let h = knots[segment + 1] - knots[segment];
        let mut bezier = [[0.0; 3]; 4];
        for axis in 0..3 {
            let mut power = [0.0f64; 4];
            for (p, slot) in power.iter_mut().enumerate() {
                *slot = entity.real(coefficients + segment * 12 + axis * 4 + p)? * h.powi(p as i32);
            }
            for k in 0..4 {
                bezier[k][axis] = (0..=k).map(|p| POWER_TO_BEZIER[k][p] * power[p]).sum();
            }
        }
        let from = if segment == 0 { 0 } else { 1 };
        control.extend_from_slice(&bezier[from..]);
    }
    let mut multiplicities = vec![3usize; segments + 1];
    multiplicities[0] = 4;
    multiplicities[segments] = 4;
    Ok((control, multiplicities, knots))
}

/// A clamped cubic knot vector on `breaks`, each interior break triple.
fn cubic_knots(breaks: &[f64]) -> Vec<f64> {
    let mut knots = Vec::with_capacity(3 * breaks.len() + 2);
    for (index, &value) in breaks.iter().enumerate() {
        let times = if index == 0 || index + 1 == breaks.len() { 4 } else { 3 };
        knots.extend(std::iter::repeat_n(value, times));
    }
    knots
}

/// The exact curve of a 130.
pub(super) enum Offset {
    Line { start: Vec3, end: Vec3 },
    /// Counter-clockwise about `axis` from `x` through `sweep` radians.
    Arc { center: Vec3, axis: Vec3, radius: f64, x: Vec3, sweep: f64 },
}

impl Offset {
    fn arc_point(center: Vec3, axis: Vec3, x: Vec3, radius: f64, angle: f64) -> Vec3 {
        let y = axis.cross(x);
        center.add(x.scale(radius * angle.cos())).add(y.scale(radius * angle.sin()))
    }
}

fn pace_name(pace: Pace) -> &'static str {
    match pace {
        Pace::Even => "a line",
        Pace::Spline => "a spline",
        Pace::Angle(_) => "an arc",
    }
}
