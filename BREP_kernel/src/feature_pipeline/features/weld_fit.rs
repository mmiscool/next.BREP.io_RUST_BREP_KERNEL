//! Joint-face fitting for fillet-weld beads along a seam of straight runs and
//! circular arcs.
//!
//! A fillet weld sits in the corner two faces make along the seam. The faces
//! may belong to one body (a concave edge) or to two bodies that touch along
//! the seam (a plate standing on another, in one part or across two placed
//! components; a tube standing on a plate). Each leg runs along its face,
//! perpendicular to the seam, on the side of the OTHER face's outward normal,
//! so the bead section is the triangle `P, P + l1·d1, P + l2·d2` inside the
//! free wedge and never enters either body.
//!
//! The section is fitted at the seam's start and carried along it: unchanged
//! along a straight run (the bead is extruded), turned about the axis along an
//! arc (the bead is revolved). A face qualifies only where the carried section
//! stays exact: its normal is constant along a straight run, and turns with
//! the arc about its axis (a plane square to the axis, a coaxial cylinder or
//! cone).
//!
//! A picked face may also stand a small root gap off a straight seam, parallel
//! to it (a sheet-metal closed corner leaves one between the flange edge and
//! the other flange's face). The section's root then moves across the gap onto
//! the picked face's plane, along the other joint face's plane, and the leg
//! that crosses the gap is backed only where its face begins.
use crate::feature_pipeline::SceneMap;
use crate::{FaceRecord, NurbsCurve, PointClass, PolygonClass, Vec2, Vec3};

/// Seam fractions sampled to decide that a face carries the seam.
const STATIONS: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
/// Faces closer than this to parallel make no corner a fillet weld can fill.
const MIN_CORNER_SIN: f64 = 0.035; // about 2 degrees

#[derive(Clone, Debug)]
pub(super) struct JointFace {
    pub name: String,
    pub solid: String,
    handle: u32,
    face_id: u64,
    /// The outward normal at the seam's start.
    normal: Vec3,
    /// For a picked face across a root gap from a straight seam: the offset
    /// from the seam to its foot on the face (constant along the seam).
    gap: Option<Vec3>,
}

impl JointFace {
    pub(super) fn same(&self, other: &JointFace) -> bool {
        same(self, other)
    }
}

/// The fitted section at the seam's start: unit leg directions along each
/// joint face and the faces they lie on, leg 1 first.
#[derive(Clone, Debug)]
pub(super) struct Fit {
    pub faces: [JointFace; 2],
    pub legs: [Vec3; 2],
    /// The offset from the seam to the section's root: zero, or across a
    /// root gap onto the gapped face's plane.
    pub root: Vec3,
    /// How far along each leg from the root its face begins (the gap a leg
    /// crosses; zero for a leg that starts on its face).
    pub skip: [f64; 2],
}

impl Fit {
    /// This fit carried `fraction` along `seam`: the section and the joint
    /// faces' normals where that fraction of the run starts.
    pub(super) fn carried(&self, seam: &Seam, fraction: f64) -> Fit {
        let face = |face: &JointFace| JointFace {
            normal: seam.carry(fraction, face.normal),
            gap: face.gap.map(|gap| seam.carry(fraction, gap)),
            ..face.clone()
        };
        Fit {
            faces: [face(&self.faces[0]), face(&self.faces[1])],
            legs: [seam.carry(fraction, self.legs[0]), seam.carry(fraction, self.legs[1])],
            root: seam.carry(fraction, self.root),
            skip: self.skip,
        }
    }
}

/// Why a seam has no fitted section. AUTO placement falls back to the manual
/// orientation on any of these; FITTED placement reports them.
pub(super) type NoFit = String;

/// How a seam segment runs, and so how a section is carried along it.
#[derive(Clone, Copy, Debug)]
pub(super) enum Run {
    /// Straight: the section is extruded along the tangent.
    Line,
    /// A circular arc of `angle` radians, right-handed about `axis` through
    /// `centre`: the section is revolved.
    Arc { centre: Vec3, axis: Vec3, angle: f64 },
}

#[derive(Clone, Debug)]
pub(super) struct Seam {
    pub start: Vec3,
    /// The unit tangent at the start.
    pub tangent: Vec3,
    pub length: f64,
    pub run: Run,
    /// The solid holding the seam edge, when the seam is a resident edge.
    pub owner: Option<String>,
}

/// `v` turned by `angle` radians right-handed about the unit `axis`.
fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    v.scale(cos).add(axis.cross(v).scale(sin)).add(axis.scale(axis.dot(v) * (1.0 - cos)))
}

impl Seam {
    pub(super) fn line(start: Vec3, tangent: Vec3, length: f64, owner: Option<String>) -> Seam {
        Seam { start, tangent, length, run: Run::Line, owner }
    }

    /// The seam a curve runs along: a straight line or a circular arc (a full
    /// circle included). `None` for any other curve.
    pub(super) fn from_curve(curve: &NurbsCurve, owner: Option<String>) -> Result<Option<Seam>, String> {
        let [t0, t1] = curve.domain()?;
        let start = curve.evaluate(t0)?;
        let end = curve.evaluate(t1)?;
        if curve.degree == 1 && curve.control_points.len() == 2 {
            let delta = end.sub(start);
            return Ok(Some(Seam::line(start, delta.normalized()?, delta.length(), owner)));
        }
        // Sampled recognition at a tolerance relative to the curve's size: an
        // arc a sketch or a primitive built is exact to rounding.
        let scale = curve.control_points.iter()
            .map(|pole| if pole.w.abs() > 1e-300 { Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w) } else { Vec3::ZERO })
            .map(|point| point.sub(start).length()).fold(1.0_f64, f64::max);
        let Some(circle) = crate::recognize_circle(curve, t0, t1, 1e-9 * scale) else { return Ok(None) };
        // The axis turns with the curve: its start tangent is `axis × radius`.
        let radial = start.sub(circle.centre);
        let tangent = curve.unit_tangent(t0, t0, t1)?;
        let axis = if circle.axis.cross(radial).dot(tangent) >= 0.0 { circle.axis } else { circle.axis.scale(-1.0) };
        let to = end.sub(circle.centre);
        let mut angle = axis.dot(radial.cross(to)).atan2(radial.dot(to));
        if angle <= 1e-12 {
            angle += std::f64::consts::TAU;
        }
        let tangent = axis.cross(radial).normalized()?;
        Ok(Some(Seam { start, tangent, length: circle.radius * angle, run: Run::Arc { centre: circle.centre, axis, angle }, owner }))
    }

    pub(super) fn at(&self, fraction: f64) -> Vec3 {
        match self.run {
            Run::Line => self.start.add(self.tangent.scale(self.length * fraction)),
            Run::Arc { centre, axis, angle } => centre.add(rotate(self.start.sub(centre), axis, angle * fraction)),
        }
    }

    /// A direction fixed in the section at the start, carried to `fraction`.
    pub(super) fn carry(&self, fraction: f64, direction: Vec3) -> Vec3 {
        match self.run {
            Run::Line => direction,
            Run::Arc { axis, angle, .. } => rotate(direction, axis, angle * fraction),
        }
    }

    /// The stretch of this run from `from` to `to` (fractions of it) as a run
    /// of its own.
    pub(super) fn part(&self, from: f64, to: f64) -> Seam {
        let run = match self.run {
            Run::Line => Run::Line,
            Run::Arc { centre, axis, angle } => Run::Arc { centre, axis, angle: angle * (to - from) },
        };
        Seam { start: self.at(from), tangent: self.carry(from, self.tangent), length: self.length * (to - from), run, owner: self.owner.clone() }
    }

    pub(super) fn end_tangent(&self) -> Vec3 {
        self.carry(1.0, self.tangent)
    }

    pub(super) fn is_line(&self) -> bool {
        matches!(self.run, Run::Line)
    }

    fn tolerance(&self) -> f64 {
        1e-6_f64.max(self.length * 1e-12)
    }
}

/// Where a straight run's bead is cut at a cornered join: the mitre plane
/// (a point and its normal, pointing along the seam) at each end, and how far
/// the run is extended past that end so the cut has material to trim.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Trim {
    pub ends: [Option<(Vec3, Vec3, f64)>; 2],
}

/// The section `points` (a closed polygon at the seam's start, square to it)
/// carried along the seam: extruded along a straight run (and mitred where
/// `trim` says), revolved along an arc.
pub(super) fn sweep(seam: &Seam, points: &[Vec3], trim: &Trim) -> Result<crate::BrepSolid, String> {
    sweep_section(seam, points, None, trim)
}

/// A circular arc from `a` to `b` in the plane of `a`, `b` and `root`,
/// bulging `height` away from `root` (negative: towards it) at its middle:
/// its centre, radius, the angle it turns through (more than a half turn
/// where the height exceeds half the chord) and the point at its middle.
pub(super) fn contour_arc(root: Vec3, a: Vec3, b: Vec3, height: f64) -> Result<(Vec3, f64, f64, Vec3), String> {
    let chord = b.sub(a);
    let half = chord.length() * 0.5;
    let middle = a.add(chord.scale(0.5));
    // Away from the root, square to the chord, in the section's plane.
    let toward = middle.sub(root);
    let out = toward.sub(chord.scale(toward.dot(chord) / chord.dot(chord))).normalized()?;
    let depth = height.abs();
    let radius = (half * half + depth * depth) / (2.0 * depth);
    let bulge = if height > 0.0 { out } else { out.scale(-1.0) };
    // The centre stands `radius − depth` back from the chord (past it, for
    // more than a half turn).
    let centre = middle.sub(bulge.scale(radius - depth));
    let angle = 2.0 * half.atan2(radius - depth);
    Ok((centre, radius, angle, middle.add(bulge.scale(depth))))
}

/// How far the bead sections (root offset, then the toe offsets from the
/// root) reach from the seam, a convex face's whole circle included (a
/// bound past the arc itself).
pub(super) fn section_reach(sections: &[[Vec3; 3]], contour: Option<f64>) -> Result<f64, String> {
    let mut reach = 0.0_f64;
    for [root, a, b] in sections {
        reach = reach.max(root.length()).max(root.add(*a).length()).max(root.add(*b).length());
        // A concave arc stays inside the section's triangle.
        if let Some(height) = contour.filter(|height| *height > 0.0) {
            let (centre, radius, _, _) = contour_arc(*root, root.add(*a), root.add(*b), height)?;
            reach = reach.max(centre.length() + radius);
        }
    }
    Ok(reach)
}

/// [`sweep`] of a three-point bead section (root, toe, toe) whose face from
/// the first toe to the second is the circular arc [`contour_arc`] of
/// `contour` height when one is given.
pub(super) fn sweep_section(seam: &Seam, points: &[Vec3], contour: Option<f64>, trim: &Trim) -> Result<crate::BrepSolid, String> {
    let mut outline = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let (from, to) = (points[i], points[(i + 1) % points.len()]);
        match contour {
            Some(height) if i == 1 && points.len() == 3 => {
                let (centre, _, _, apex) = contour_arc(points[0], from, to, height)?;
                outline.push(Side::Arc { from, to, centre, through: apex });
            }
            _ => outline.push(Side::Line(from, to)),
        }
    }
    sweep_outline(seam, &outline, trim)
}

/// One side of a closed section outline at a run's start, square to it.
#[derive(Clone, Copy, Debug)]
pub(super) enum Side {
    Line(Vec3, Vec3),
    /// A circular arc about `centre` from `from` to `to`, through `through`
    /// (any point of it strictly between the ends).
    Arc { from: Vec3, to: Vec3, centre: Vec3, through: Vec3 },
}

impl Side {
    fn moved(self, shift: Vec3) -> Side {
        match self {
            Side::Line(a, b) => Side::Line(a.add(shift), b.add(shift)),
            Side::Arc { from, to, centre, through } => Side::Arc { from: from.add(shift), to: to.add(shift), centre: centre.add(shift), through: through.add(shift) },
        }
    }

    fn curve(self) -> Result<NurbsCurve, String> {
        match self {
            Side::Line(a, b) => crate::make_line(a, b),
            Side::Arc { from, to, centre, through } => {
                let radius = from.sub(centre).length();
                let x_axis = from.sub(centre).normalized()?;
                // The turn from `from` that passes `through`: its plane's
                // normal, then the angle to `to` measured that way round.
                let mut normal = x_axis.cross(through.sub(centre));
                let toward = to.sub(centre);
                if normal.length() <= 1e-12 * radius {
                    return Err("an arc's middle point is opposite its start".into());
                }
                normal = normal.normalized()?;
                let y_axis = normal.cross(x_axis);
                let mut angle = toward.dot(y_axis).atan2(toward.dot(x_axis));
                if angle <= 0.0 {
                    angle += std::f64::consts::TAU;
                }
                crate::make_arc(centre, x_axis, y_axis, radius, 0.0, angle)
            }
        }
    }
}

/// The closed outline `sides` carried along the seam: extruded along a
/// straight run (and mitred where `trim` says), revolved along an arc.
pub(super) fn sweep_outline(seam: &Seam, sides: &[Side], trim: &Trim) -> Result<crate::BrepSolid, String> {
    match seam.run {
        Run::Line => {
            let before = trim.ends[0].map_or(0.0, |(_, _, extend)| extend);
            let after = trim.ends[1].map_or(0.0, |(_, _, extend)| extend);
            let shift = seam.tangent.scale(-before);
            let profile = sides.iter().map(|side| side.moved(shift).curve()).collect::<Result<Vec<_>, _>>()?;
            let mut solid = crate::extrude_profile_brep(&profile, seam.tangent, seam.length + before + after)
                .map_err(|error| error.to_string())?;
            for (end, cut) in trim.ends.iter().enumerate() {
                let Some((point, normal, _)) = cut else { continue };
                let (below, above) = crate::split_solid_by_plane(&solid, *point, *normal)
                    .map_err(|error| format!("could not mitre the bead: {error}"))?;
                // The start cut keeps what lies ahead of it, the end cut what lies behind.
                solid = if end == 0 { above } else { below };
            }
            Ok(solid)
        }
        Run::Arc { centre, axis, angle } => {
            if trim.ends.iter().any(Option::is_some) {
                return Err("an arc is never mitred".into());
            }
            let profile = sides.iter().map(|side| side.curve()).collect::<Result<Vec<_>, _>>()?;
            crate::revolve_profile_brep(&profile, centre, axis, angle)
        }
    }
}

fn with_face<T>(handle: u32, face_id: u64, f: impl FnOnce(&FaceRecord) -> Result<T, String>) -> Result<T, String> {
    crate::with_registered_solid_str(handle, |solid| {
        let face = solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == face_id)
            .ok_or_else(|| format!("face {face_id} is no longer resident"))?;
        f(face)
    })
}

fn face_class(face: &FaceRecord, point: Vec3, tolerance: f64) -> Result<Option<PolygonClass>, String> {
    let projection = crate::project_point_to_surface(&face.surface, point)?;
    if projection.distance > tolerance {
        return Ok(None);
    }
    crate::parameter_point_in_face(face, Vec2 { x: projection.u, y: projection.v }, tolerance).map(Some)
}

/// The face's outward normal at the seam's start if the seam lies on it along
/// its whole length, inside or on its trim, and the normal is carried with the
/// section: constant along a straight run, turning about an arc's axis.
fn carries_seam(face: &FaceRecord, seam: &Seam) -> Result<Option<Vec3>, String> {
    let tolerance = seam.tolerance();
    let mut normal: Option<Vec3> = None;
    for fraction in STATIONS {
        let point = seam.at(fraction);
        if !matches!(face_class(face, point, tolerance)?, Some(PolygonClass::Inside | PolygonClass::Boundary)) {
            return Ok(None);
        }
        let (_, outward) = crate::fillet::face_plane_normal(face, point).map_err(|refusal| refusal.to_string())?;
        match normal {
            None => normal = Some(outward),
            // A face that turns against the section is not a joint face.
            Some(first) => {
                let carried = seam.carry(fraction, first);
                if carried.cross(outward).length() > 1e-9 || carried.dot(outward) < 0.0 {
                    return Ok(None);
                }
            }
        }
    }
    Ok(normal)
}

/// An axis-aligned box around the body. A NURBS curve or surface lies in the
/// convex hull of its poles, so the poles of every edge and every face carrier
/// bound the body, curved face interiors included.
fn bounds(solid: &crate::BrepSolid) -> (Vec3, Vec3) {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let edge_poles = solid.edges.iter().flat_map(|edge| edge.curve.control_points.iter());
    let face_poles = solid.shells.iter().flat_map(|shell| &shell.faces)
        .flat_map(|face| face.surface.control_points.iter().flatten());
    for pole in edge_poles.chain(face_poles) {
        let w = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
        let p = Vec3::new(pole.x / w, pole.y / w, pole.z / w);
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    (low, high)
}

fn solid_bounds_contain(solid: &crate::BrepSolid, points: &[Vec3], tolerance: f64) -> bool {
    let (low, high) = bounds(solid);
    points.iter().all(|p| {
        p.x >= low.x - tolerance && p.y >= low.y - tolerance && p.z >= low.z - tolerance
            && p.x <= high.x + tolerance && p.y <= high.y + tolerance && p.z <= high.z + tolerance
    })
}

/// A bead built by a weld feature (every face a bead face) offers no joint
/// faces to another weld; a body a bead was merged into still does. Every
/// body, beads included, is an obstacle to the bead.
fn is_weld_bead(solid: &crate::BrepSolid) -> bool {
    solid.shells.iter().flat_map(|shell| &shell.faces)
        .all(|face| face.name.as_deref().is_some_and(|name| name.contains(":BeadFace")))
}

/// Whether scene solid `name` is one of weld `weld`'s own beads (`{weld}`,
/// or `{weld}:Bead{n}` for a bead in several bodies).
pub(super) fn owned_by(name: &str, weld: &str) -> bool {
    name == weld || name.strip_prefix(weld).is_some_and(|rest| rest.starts_with(":Bead"))
}

/// Every resident face that carries the seam, in scene-name order.
fn candidates(scene: &SceneMap, seam: &Seam, exclude: &str) -> Result<Vec<JointFace>, String> {
    let ends = [seam.at(0.0), seam.at(1.0)];
    let mut names: Vec<(&String, &u32)> = scene.solids.iter().filter(|(name, _)| !owned_by(name, exclude)).collect();
    names.sort();
    let mut found = Vec::new();
    for (solid_name, &handle) in names {
        let faces = crate::with_registered_solid_str(handle, |solid| {
            if is_weld_bead(solid) || !solid_bounds_contain(solid, &ends, seam.tolerance()) {
                return Ok(Vec::new());
            }
            let mut faces = Vec::new();
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                // A face this check cannot read (a degenerate normal at a pole,
                // a failed projection) is not a joint face; it is never fatal.
                if let Ok(Some(normal)) = carries_seam(face, seam) {
                    faces.push((face.id, face.name.clone(), normal));
                }
            }
            Ok(faces)
        })?;
        for (face_id, name, normal) in faces {
            let name = name.unwrap_or_else(|| format!("{solid_name} face {face_id}"));
            found.push(JointFace { name, solid: solid_name.clone(), handle, face_id, normal, gap: None });
        }
    }
    Ok(found)
}

/// A picked flat face standing a root gap off a straight seam: parallel to it,
/// the seam on its free side at one distance along the whole seam, and the gap
/// narrower than `widest`. `None` for any other face. Where on the face the
/// root lands depends on the other joint face (an oblique one moves it off the
/// seam's perpendicular foot), so the fit's corner probe, toe checks and the
/// support check decide that the face holds the root and its leg.
fn across_gap(scene: &SceneMap, seam: &Seam, name: &str, widest: f64) -> Result<Option<JointFace>, String> {
    if !seam.is_line() {
        return Ok(None);
    }
    let tolerance = seam.tolerance();
    for face in scene.resolve_face_group(name) {
        let found = with_face(face.handle, face.face_id, |record| {
            let mut first: Option<(Vec3, Vec3)> = None;
            for fraction in STATIONS {
                let point = seam.at(fraction);
                // The foot on the face's plane, not on its bounded carrier
                // (a projection there clamps to the patch's edge).
                let Ok((on, outward)) = crate::fillet::face_plane_normal(record, point) else { return Ok(None) };
                // Parallel to the seam, the seam outside the face.
                let gap = point.sub(on).dot(outward);
                if outward.dot(seam.tangent).abs() > 1e-9 || gap <= tolerance || gap >= widest {
                    return Ok(None);
                }
                // Flat: every pole of the carrier lies on that plane.
                let flat = record.surface.control_points.iter().flatten().all(|pole| {
                    let w = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
                    Vec3::new(pole.x / w, pole.y / w, pole.z / w).sub(on).dot(outward).abs() <= tolerance
                });
                if !flat {
                    return Ok(None);
                }
                let offset = outward.scale(-gap);
                match first {
                    None => first = Some((offset, outward)),
                    Some((offset0, outward0)) => {
                        if offset.sub(offset0).length() > tolerance || outward.cross(outward0).length() > 1e-9 {
                            return Ok(None);
                        }
                    }
                }
            }
            Ok(first)
        })?;
        if let Some((offset, normal)) = found {
            let solid = scene.solids.iter().filter(|(_, handle)| **handle == face.handle)
                .map(|(name, _)| name.clone()).min().unwrap_or_default();
            return Ok(Some(JointFace { name: name.to_string(), solid, handle: face.handle, face_id: face.face_id, normal, gap: Some(offset) }));
        }
    }
    Ok(None)
}

/// Whether the named face stands a bridgeable root gap off a straight seam.
pub(super) fn bridges(scene: &SceneMap, seam: &Seam, name: &str, legs: [f64; 2]) -> Result<bool, String> {
    Ok(across_gap(scene, seam, name, legs[0].max(legs[1]))?.is_some())
}

/// The section's root for a pair: the offset from the seam to it and how far
/// each leg runs before its face begins. Across a root gap the root moves
/// along the face carrying the seam onto the gapped face's plane, backwards
/// along that face's leg; `None` when it cannot (both faces gapped, or the
/// faces too near parallel to meet).
fn root_of(pair: [&JointFace; 2], tangent: Vec3) -> Result<Option<(Vec3, [f64; 2])>, String> {
    let (gapped, carrier) = match (pair[0].gap, pair[1].gap) {
        (None, None) => return Ok(Some((Vec3::ZERO, [0.0, 0.0]))),
        (Some(_), None) => (0, 1),
        (None, Some(_)) => (1, 0),
        (Some(_), Some(_)) => return Ok(None),
    };
    let offset = pair[gapped].gap.expect("the gapped face");
    let normal = pair[gapped].normal;
    let leg = leg_direction(pair[carrier], pair[gapped], tangent)?;
    let across = leg.dot(normal);
    if across.abs() < MIN_CORNER_SIN {
        return Ok(None);
    }
    let shift = leg.scale(offset.dot(normal) / across);
    // The carrying face's leg runs from the root back across the gap.
    if shift.dot(leg) >= 0.0 {
        return Ok(None);
    }
    let mut skip = [0.0; 2];
    skip[carrier] = shift.length();
    Ok(Some((shift, skip)))
}

/// The leg direction along `face`, on the free side of `other`.
fn leg_direction(face: &JointFace, other: &JointFace, tangent: Vec3) -> Result<Vec3, String> {
    let direction = face.normal.cross(tangent).normalized()?;
    Ok(if direction.dot(other.normal) < 0.0 { direction.scale(-1.0) } else { direction })
}

fn runs_onto(face: &JointFace, point: Vec3, tolerance: f64) -> Result<bool, String> {
    with_face(face.handle, face.face_id, |record| {
        Ok(matches!(face_class(record, point, tolerance)?, Some(PolygonClass::Inside | PolygonClass::Boundary)))
    })
}

fn outside_solid(scene: &SceneMap, solid: &str, point: Vec3, tolerance: f64) -> Result<bool, String> {
    let handle = scene.solids.get(solid).copied().ok_or_else(|| format!("solid '{solid}' is no longer resident"))?;
    crate::with_registered_solid_str(handle, |body| {
        Ok(crate::classify_point(point, body, tolerance)?.class == PointClass::Out)
    })
}

/// Check the pair makes a corner with free space between the legs: both short
/// legs land on their faces and the wedge between them is outside every body
/// that offered a joint face.
fn corner(scene: &SceneMap, seam: &Seam, pair: [&JointFace; 2], solids: &[String], probe: f64) -> Result<Option<[Vec3; 2]>, String> {
    if pair[0].normal.cross(pair[1].normal).length() < MIN_CORNER_SIN {
        return Ok(None);
    }
    let legs = [leg_direction(pair[0], pair[1], seam.tangent)?, leg_direction(pair[1], pair[0], seam.tangent)?];
    let Some((root, skip)) = root_of(pair, seam.tangent)? else { return Ok(None) };
    let tolerance = seam.tolerance();
    let middle = seam.at(0.5).add(seam.carry(0.5, root));
    let carried = [seam.carry(0.5, legs[0]), seam.carry(0.5, legs[1])];
    for ((face, leg), skip) in pair.iter().zip(&carried).zip(skip) {
        if !runs_onto(face, middle.add(leg.scale(skip + probe)), tolerance)? {
            return Ok(None);
        }
    }
    let wedge = middle.add(carried[0].add(carried[1]).scale(probe / 3.0));
    for solid in solids {
        if !outside_solid(scene, solid, wedge, tolerance * 0.1)? {
            return Ok(None);
        }
    }
    Ok(Some(legs))
}

/// Find the joint along a straight or circular seam. `explicit` holds the faces the user
/// selected (in selection order, at most two). `legs` are the requested leg
/// sizes; the toe of each leg must stay on its face along the whole seam.
/// `previous`, the fit of the run before on a chain, keeps leg 1 on the face
/// leg 1 ran along there (checked before the leg sizes are).
pub(super) fn fit(scene: &SceneMap, seam: &Seam, explicit: &[String], exclude: &str, legs: [f64; 2], previous: Option<&Fit>) -> Result<Result<Fit, NoFit>, String> {
    if explicit.len() > 2 {
        return Err("select at most two joint faces".into());
    }
    let all = candidates(scene, seam, exclude)?;
    let mut chosen = Vec::new();
    for name in explicit {
        // A container name picks the member that carries the seam.
        let group = scene.resolve_face_group(name);
        if let Some(joint) = all.iter().find(|joint| group.iter().any(|face| joint.handle == face.handle && joint.face_id == face.face_id)) {
            chosen.push(JointFace { name: name.clone(), ..joint.clone() });
            continue;
        }
        // A flat face a root gap off a straight seam, narrower than the legs.
        if let Some(joint) = across_gap(scene, seam, name, legs[0].max(legs[1]))? {
            chosen.push(joint);
            continue;
        }
        return Ok(Err(format!("joint face '{name}' does not carry the seam along its whole length {}",
            if seam.is_line() { "as a flat face" } else { "square to the arc's axis or coaxial with it" })));
    }
    let mut solids: Vec<String> = all.iter().chain(chosen.iter()).map(|joint| joint.solid.clone()).collect();
    solids.sort();
    solids.dedup();
    // A picked face across a root gap pairs with the faces carrying the seam.
    let pool: Vec<JointFace> = all.iter().cloned().chain(chosen.iter().filter(|joint| joint.gap.is_some()).cloned()).collect();
    let probe = (legs[0].min(legs[1]) * 0.05).min(seam.length * 0.05).max(seam.tolerance() * 10.0);
    let mut pairs: Vec<([JointFace; 2], [Vec3; 2])> = Vec::new();
    match chosen.as_slice() {
        [first, second] => {
            if let Some(legs) = corner(scene, seam, [first, second], &solids, probe)? {
                pairs.push(([first.clone(), second.clone()], legs));
            }
        }
        _ => {
            for (i, a) in pool.iter().enumerate() {
                for b in &pool[i + 1..] {
                    if let [only] = chosen.as_slice() {
                        if !(same(only, a) || same(only, b)) {
                            continue;
                        }
                    }
                    if corner(scene, seam, [a, b], &solids, probe)?.is_some() {
                        let [f0, f1] = order(seam, chosen.first(), [a, b]);
                        let legs = [leg_direction(&f0, &f1, seam.tangent)?, leg_direction(&f1, &f0, seam.tangent)?];
                        pairs.push(([f0, f1], legs));
                    }
                }
            }
        }
    }
    let ([mut first, mut second], mut legs_dir) = match pairs.len() {
        0 if all.is_empty() => return Ok(Err(if seam.is_line() {
            "no face carries the seam as a flat face".into()
        } else {
            "no face carries the arc square to its axis or coaxial with it".into()
        })),
        0 => return Ok(Err(format!(
            "no concave corner along the seam between faces {}", list(&all)))),
        1 => pairs.pop().expect("one pair"),
        _ => return Ok(Err(format!(
            "several corners meet at the seam ({}); select the two joint faces",
            pairs.iter().map(|(faces, _)| format!("{} + {}", faces[0].name, faces[1].name)).collect::<Vec<_>>().join(", ")))),
    };
    if let Some(previous) = previous {
        if first.same(&previous.faces[1]) || second.same(&previous.faces[0]) {
            std::mem::swap(&mut first, &mut second);
            legs_dir.swap(0, 1);
        }
    }
    let tolerance = seam.tolerance();
    let Some((root, skip)) = root_of([&first, &second], seam.tangent)? else {
        return Ok(Err(format!("no root across the gap between faces {} and {}", first.name, second.name)));
    };
    for (((face, leg), size), skip) in [&first, &second].iter().zip(&legs_dir).zip(legs).zip(skip) {
        if size <= skip + tolerance {
            return Ok(Err(format!("leg {size} does not reach across the {skip} root gap onto joint face {}", face.name)));
        }
        for fraction in STATIONS {
            let toe = seam.at(fraction).add(seam.carry(fraction, root.add(leg.scale(size))));
            if !runs_onto(face, toe, tolerance)? {
                return Ok(Err(format!("leg {size} runs off joint face {}", face.name)));
            }
        }
    }
    Ok(Ok(Fit { faces: [first, second], legs: legs_dir, root, skip }))
}

/// The far side of a fitted joint on one run: the mirror corner across the
/// member standing on the joint (a T joint's web, a tube's wall). One joint
/// face is the member's; the nearest face of the member's body straight
/// across it (its outward normal opposite, at the member's thickness) meets
/// the other, through face along the run carried across that thickness.
/// Returned as that run and its fit; leg 1 stays on the through face when it
/// is there on the near side, else on the member's far face. Each joint face
/// is tried as the member; exactly one may give a far corner.
pub(super) fn other_side(scene: &SceneMap, seam: &Seam, near: &Fit, exclude: &str, legs: [f64; 2]) -> Result<Result<(Seam, Fit), NoFit>, String> {
    if near.faces.iter().any(|face| face.gap.is_some()) {
        return Ok(Err("the joint has a root gap; weld the far side's seam as its own weld".into()));
    }
    let tolerance = seam.tolerance();
    let mut found: Vec<(Seam, Fit)> = Vec::new();
    for member in 0..2 {
        let face = &near.faces[member];
        let through = &near.faces[1 - member];
        // A point on the member's face halfway along the seam and up its leg.
        let point = seam.at(0.5).add(seam.carry(0.5, near.root.add(near.legs[member].scale(legs[member] * 0.5))));
        let inward = seam.carry(0.5, face.normal).scale(-1.0);
        let far = crate::with_registered_solid_str(face.handle, |solid| {
            let mut best: Option<(f64, u64)> = None;
            for record in solid.shells.iter().flat_map(|shell| &shell.faces) {
                if record.id == face.face_id {
                    continue;
                }
                let Ok((foot, outward)) = crate::fillet::face_plane_normal(record, point) else { continue };
                let across = foot.sub(point);
                let thickness = across.dot(inward);
                if thickness <= tolerance || across.sub(inward.scale(thickness)).length() > tolerance
                    || outward.dot(inward) < 1.0 - 1e-9 {
                    continue;
                }
                if !matches!(face_class(record, foot, tolerance)?, Some(PolygonClass::Inside | PolygonClass::Boundary)) {
                    continue;
                }
                if best.is_none_or(|(nearest, _)| thickness < nearest) {
                    best = Some((thickness, record.id));
                }
            }
            Ok(best)
        })?;
        let Some((thickness, far_id)) = far else { continue };
        // The seam carried across the member: along an arc the far run is
        // the coaxial arc at the far face's radius.
        let shift = face.normal.scale(-thickness);
        let start = seam.start.add(shift);
        let far_seam = match seam.run {
            Run::Line => Seam::line(start, seam.tangent, seam.length, None),
            Run::Arc { centre, axis, angle } => {
                let radial = start.sub(centre);
                let radius = radial.sub(axis.scale(radial.dot(axis))).length();
                if radius <= tolerance {
                    continue;
                }
                Seam { start, tangent: seam.tangent, length: radius * angle, run: seam.run, owner: None }
            }
        };
        let joints = candidates(scene, &far_seam, exclude)?;
        let Some(far_face) = joints.iter().find(|joint| joint.handle == face.handle && joint.face_id == far_id) else { continue };
        let Some(through_face) = joints.iter().find(|joint| same(joint, through)) else { continue };
        let picks = if member == 0 { [far_face.name.clone(), through_face.name.clone()] } else { [through_face.name.clone(), far_face.name.clone()] };
        match fit(scene, &far_seam, &picks, exclude, legs, None)? {
            Ok(joint) => found.push((far_seam, joint)),
            Err(_) => continue,
        }
    }
    Ok(match found.len() {
        0 => Err("no far-side corner: neither joint face belongs to a member with a face straight across it on the other face".into()),
        1 => Ok(found.pop().expect("one far side")),
        _ => Err("both joint faces belong to members that continue past the joint; weld the far side's seam as its own weld".into()),
    })
}

/// Does the named face carry the whole seam (on or inside its trim)?
pub(super) fn carries(scene: &SceneMap, seam: &Seam, name: &str) -> Result<bool, String> {
    for face in scene.resolve_face_group(name) {
        if with_face(face.handle, face.face_id, |record| carries_any(record, seam))? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn carries_any(face: &FaceRecord, seam: &Seam) -> Result<bool, String> {
    let tolerance = seam.tolerance();
    for fraction in STATIONS {
        if !matches!(face_class(face, seam.at(fraction), tolerance)?, Some(PolygonClass::Inside | PolygonClass::Boundary)) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn same(a: &JointFace, b: &JointFace) -> bool {
    a.handle == b.handle && a.face_id == b.face_id
}

fn list(faces: &[JointFace]) -> String {
    faces.iter().map(|face| face.name.as_str()).collect::<Vec<_>>().join(", ")
}

/// Leg 1 goes on the selected face, else on the seam owner's face, else on
/// the face whose name sorts first, so unequal legs never swap on rebuild.
fn order(seam: &Seam, selected: Option<&JointFace>, pair: [&JointFace; 2]) -> [JointFace; 2] {
    let first_wins = match selected {
        Some(selected) => same(selected, pair[0]) || !same(selected, pair[1]),
        None => {
            let owned = |face: &JointFace| seam.owner.as_deref() == Some(face.solid.as_str());
            match (owned(pair[0]), owned(pair[1])) {
                (true, false) => true,
                (false, true) => false,
                _ => (&pair[0].name, &pair[0].solid) <= (&pair[1].name, &pair[1].solid),
            }
        }
    };
    if first_wins { [pair[0].clone(), pair[1].clone()] } else { [pair[1].clone(), pair[0].clone()] }
}

/// Joins closer to straight than this (1 − cos of the turn) are tangent: the
/// two segments share the section there.
const TANGENT_JOIN: f64 = 1e-9;
/// A mitre plane must cut the runs it trims at least this squarely (cosine of
/// half the turn): past it the seam folds back on itself (about 168.5°, the
/// sweep's own mitre limit).
const MITRE_FLOOR: f64 = 0.1;

/// A seam of straight runs and circular arcs, head to tail, with the mitre
/// plane at each cornered join.
pub(super) struct Chain {
    pub runs: Vec<Seam>,
    /// `corners[k]`: the join after run `k` (the closing one last on a closed
    /// chain) as the mitre plane's point and forward normal, or `None` where
    /// the runs meet tangent.
    corners: Vec<(usize, usize, Option<(Vec3, Vec3)>)>,
}

impl Chain {
    /// Classify the joins of `runs` (ordered head to tail). A corner is
    /// mitred between two straight runs; a corner at an arc, or a turn that
    /// folds the seam back on itself, has no bead.
    pub(super) fn new(runs: Vec<Seam>) -> Result<Chain, NoFit> {
        let count = runs.len();
        let tolerance = runs.iter().map(Seam::tolerance).fold(0.0, f64::max);
        let closed = count > 1 && runs[count - 1].at(1.0).sub(runs[0].start).length() <= tolerance * 10.0;
        let mut corners = Vec::new();
        let joins: Vec<(usize, usize)> = (0..count.saturating_sub(1)).map(|k| (k, k + 1))
            .chain(closed.then(|| (count - 1, 0))).collect();
        let name = |k: usize| k + 1;
        for (a, b) in joins {
            let (out, into) = (runs[a].end_tangent(), runs[b].tangent);
            if out.dot(into) >= 1.0 - TANGENT_JOIN {
                corners.push((a, b, None));
                continue;
            }
            if !(runs[a].is_line() && runs[b].is_line()) {
                return Err(format!("segments {} and {} meet at a corner on an arc; a bead is mitred only between straight segments", name(a), name(b)));
            }
            let normal = out.add(into).normalized().map_err(|_| format!("the seam doubles back where segments {} and {} meet", name(a), name(b)))?;
            if out.dot(normal) < MITRE_FLOOR {
                return Err(format!("the seam turns back too sharply where segments {} and {} meet for a mitred bead", name(a), name(b)));
            }
            corners.push((a, b, Some((runs[b].start, normal))));
        }
        Ok(Chain { runs, corners })
    }

    pub(super) fn is_single_line(&self) -> bool {
        self.runs.len() == 1 && self.runs[0].is_line()
    }

    fn label(&self, k: usize) -> String {
        if self.runs.len() == 1 { String::new() } else { format!("segment {}: ", k + 1) }
    }

    /// The mitre cut at each end of each run, for a section reaching `reach`
    /// from the seam. A run too short for its two mitres has no bead.
    pub(super) fn trims(&self, reach: f64) -> Result<Vec<Trim>, NoFit> {
        let mut trims = vec![Trim::default(); self.runs.len()];
        let mut taken = vec![0.0_f64; self.runs.len()];
        for (a, b, corner) in &self.corners {
            let Some((point, normal)) = corner else { continue };
            let cos_half = self.runs[*b].tangent.dot(*normal);
            // A section point `w` off the seam meets the plane `w·tan(θ/2)`
            // from the join at most; extend past that before cutting.
            let tan_half = (1.0 - cos_half * cos_half).max(0.0).sqrt() / cos_half;
            let extend = reach * tan_half * 2.0 + reach * 0.01;
            trims[*a].ends[1] = Some((*point, *normal, extend));
            trims[*b].ends[0] = Some((*point, *normal, extend));
            taken[*a] += reach * tan_half;
            taken[*b] += reach * tan_half;
        }
        for (k, run) in self.runs.iter().enumerate() {
            if taken[k] >= run.length {
                return Err(format!("{}the segment is too short for the bead's mitred corners", self.label(k)));
            }
        }
        Ok(trims)
    }

    /// Fit every run. Leg 1 stays on the same face from run to run (fitting
    /// starts at the run carrying the first picked face and works outward
    /// both ways, each run ordered by its fitted neighbour), and where runs
    /// meet tangent their sections must agree.
    pub(super) fn fit(&self, scene: &SceneMap, explicit: &[String], exclude: &str, legs: [f64; 2]) -> Result<Result<Vec<Fit>, NoFit>, String> {
        let count = self.runs.len();
        // On a chain a picked face joins the runs it carries.
        let mut picked: Vec<Vec<String>> = Vec::with_capacity(count);
        for run in &self.runs {
            if count == 1 {
                picked.push(explicit.to_vec());
                continue;
            }
            let mut names = Vec::new();
            for name in explicit {
                if carries(scene, run, name)? {
                    names.push(name.clone());
                }
            }
            picked.push(names);
        }
        let start = explicit.first()
            .and_then(|first| picked.iter().position(|names| names.first() == Some(first)))
            .unwrap_or(0);
        let order: Vec<(usize, Option<usize>)> = std::iter::once((start, None))
            .chain((start + 1..count).map(|k| (k, Some(k - 1))))
            .chain((0..start).rev().map(|k| (k, Some(k + 1))))
            .collect();
        let mut fitted: Vec<Option<Fit>> = vec![None; count];
        for (k, neighbour) in order {
            let next = neighbour.and_then(|n| fitted[n].as_ref());
            match fit(scene, &self.runs[k], &picked[k], exclude, legs, next)? {
                Ok(joint) => fitted[k] = Some(joint),
                Err(reason) => return Ok(Err(format!("{}{reason}", self.label(k)))),
            }
        }
        let fits: Vec<Fit> = fitted.into_iter().map(|fit| fit.expect("every run fitted")).collect();
        for (a, b, corner) in &self.corners {
            if corner.is_some() {
                continue;
            }
            let tolerance = self.runs[*a].tolerance().max(self.runs[*b].tolerance()) * 10.0;
            let roots = [self.runs[*a].carry(1.0, fits[*a].root), fits[*b].root];
            if roots[0].sub(roots[1]).length() > tolerance {
                return Ok(Err(format!("the joint changes where segments {} and {} meet", a + 1, b + 1)));
            }
            for (i, size) in legs.iter().enumerate() {
                let end = self.runs[*a].carry(1.0, fits[*a].legs[i]).scale(*size);
                let start = fits[*b].legs[i].scale(*size);
                if end.sub(start).length() > tolerance {
                    return Ok(Err(format!("the joint changes where segments {} and {} meet", a + 1, b + 1)));
                }
            }
        }
        Ok(Ok(fits))
    }

    /// The seam's length along all its runs.
    pub(super) fn length(&self) -> f64 {
        self.runs.iter().map(|run| run.length).sum()
    }

    /// Whether the last run ends where the first starts.
    pub(super) fn is_closed(&self) -> bool {
        self.corners.iter().any(|(a, b, _)| *b == 0 && *a + 1 == self.runs.len() && self.runs.len() > 1)
    }

    /// The pieces of the bead along the stretch `from..to` of the seam (by
    /// length from its start): the part of every run the stretch overlaps,
    /// with that run's section carried to where the part starts. A mitre cut
    /// is kept where the stretch runs on through its corner (and, for the
    /// whole of a closed seam, at the corner where it closes); the stretch's
    /// own ends are square to the seam, at a corner too.
    pub(super) fn pieces(&self, sections: &[[Vec3; 3]], trims: &[Trim], from: f64, to: f64) -> Vec<Piece> {
        let mut pieces = Vec::new();
        let mut start = 0.0;
        let total = self.length();
        let last = self.runs.len().saturating_sub(1);
        let closing = self.is_closed() && from <= self.runs[0].tolerance() && to >= total - self.runs[last].tolerance();
        for (k, run) in self.runs.iter().enumerate() {
            let end = start + run.length;
            let (low, high) = (from.max(start), to.min(end));
            let tolerance = run.tolerance();
            if high - low > tolerance {
                let f0 = if low - start <= tolerance { 0.0 } else { (low - start) / run.length };
                let f1 = if end - high <= tolerance { 1.0 } else { (high - start) / run.length };
                let [root, a, b] = sections[k];
                let through_start = from < start - tolerance || (closing && k == 0);
                let through_end = to > end + tolerance || (closing && k == last);
                let trim = Trim { ends: [
                    if f0 == 0.0 && through_start { trims[k].ends[0] } else { None },
                    if f1 == 1.0 && through_end { trims[k].ends[1] } else { None },
                ] };
                pieces.push(Piece {
                    run: run.part(f0, f1),
                    section: [run.carry(f0, root), run.carry(f0, a), run.carry(f0, b)],
                    trim,
                    index: k,
                    from: f0,
                });
            }
            start = end;
        }
        pieces
    }

    /// The exact checks on one body of pieces of a fitted bead: clear of
    /// every body, and each piece's legs backed along it.
    pub(super) fn verify_pieces(&self, scene: &SceneMap, fits: &[Fit], legs: [f64; 2], bead: &crate::BrepSolid, pieces: &[Piece], exclude: &str) -> Result<Result<(), NoFit>, String> {
        let tolerance = self.runs.iter().map(Seam::tolerance).fold(0.0, f64::max);
        if let Err(reason) = clear(scene, bead, exclude, tolerance)? {
            return Ok(Err(reason));
        }
        for piece in pieces {
            let joint = fits[piece.index].carried(&self.runs[piece.index], piece.from);
            if let Err(reason) = supported(&piece.run, &joint, legs, &piece.trim)? {
                return Ok(Err(format!("{}{reason}", self.label(piece.index))));
            }
        }
        Ok(Ok(()))
    }
}

/// One piece of a bead: a run (or a stretch of one), the section at its
/// start (the root's offset from the start, then the two toe offsets from
/// the root), its cuts, and which seam run it comes from and where on it.
#[derive(Clone, Debug)]
pub(super) struct Piece {
    pub run: Seam,
    pub section: [Vec3; 3],
    pub trim: Trim,
    pub index: usize,
    pub from: f64,
}

/// The pieces swept and united into one body, its faces named
/// `{id}:BeadFace{n}`; each face between the toes is the circular arc of
/// `contour` height when one is given.
pub(super) fn build_bead(id: &str, pieces: &[Piece], contour: Option<f64>) -> Result<crate::BrepSolid, String> {
    let mut bead: Option<crate::BrepSolid> = None;
    for Piece { run, section: [root, a, b], trim, .. } in pieces {
        // Keep the section counter-clockwise about the seam.
        let (a, b) = if a.cross(*b).dot(run.tangent) >= 0.0 { (*a, *b) } else { (*b, *a) };
        let start = run.start.add(*root);
        let piece = sweep_section(run, &[start, start.add(a), start.add(b)], contour, trim)?;
        bead = Some(match bead {
            None => piece,
            Some(sum) => crate::boolean_operation(&sum, &piece, crate::BooleanOperation::Union, &crate::BooleanOptions::default())
                .map_err(|refusal| format!("could not join the bead's segments: {refusal}"))?,
        });
    }
    let mut bead = bead.ok_or("the seam is empty")?;
    for (i, face) in bead.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
        face.name = Some(format!("{id}:BeadFace{}", i + 1));
    }
    Ok(bead)
}

impl Chain {
    /// The normal of the plane every run lies in (an arc's axis), for placing
    /// a section by Orientation; `None` when the runs do not share one.
    pub(super) fn plane(&self) -> Option<Vec3> {
        let mut up: Option<Vec3> = None;
        for run in &self.runs {
            if let Run::Arc { axis, .. } = run.run {
                up.get_or_insert(axis);
            }
        }
        if up.is_none() {
            for pair in self.runs.windows(2) {
                if let Ok(normal) = pair[0].tangent.cross(pair[1].tangent).normalized() {
                    up = Some(normal);
                    break;
                }
            }
        }
        // Every run along one line: any normal of it will do.
        let up = match up {
            Some(up) => up,
            None => self.runs[0].tangent.perpendicular().ok()?,
        };
        let origin = self.runs[0].start;
        let flat = self.runs.iter().all(|run| {
            let tolerance = run.tolerance() * 10.0;
            let axis_ok = match run.run {
                Run::Line => run.tangent.dot(up).abs() <= 1e-9,
                Run::Arc { axis, .. } => axis.cross(up).length() <= 1e-9,
            };
            axis_ok && run.start.sub(origin).dot(up).abs() <= tolerance && run.at(1.0).sub(origin).dot(up).abs() <= tolerance
        });
        flat.then_some(up)
    }
}

/// The volume (cubic model units) a clearance or support boolean may leave
/// and still read as empty: fixed, so a large bead never hides a small defect.
const LEFTOVER_VOLUME: f64 = 1e-9;

/// A yes/no boolean: the result is only measured, so coplanar fragments are
/// left unmerged.
fn boolean(a: &crate::BrepSolid, b: &crate::BrepSolid, operation: crate::BooleanOperation) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    let options = crate::BooleanOptions { merge_coplanar_faces: false, ..Default::default() };
    crate::boolean_operation(a, b, operation, &options)
}

/// An empty result where the operands only touch: the boolean refuses it as a
/// conservative empty overlap rather than returning no body.
fn empty(refusal: &crate::KernelRefusal) -> bool {
    matches!(refusal.class, crate::RefusalClass::ConservativeEmptyOverlap)
}

/// The volume two bodies share; a touching-only pair shares none.
pub(super) fn common_volume(a: &crate::BrepSolid, b: &crate::BrepSolid) -> Result<f64, NoFit> {
    match boolean(a, b, crate::BooleanOperation::Intersect) {
        Ok(common) => crate::solid_mass_properties(&common).map(|m| m.volume.abs())
            .map_err(|error| format!("could not measure the common part: {error}")),
        Err(refusal) if empty(&refusal) => Ok(0.0),
        Err(refusal) => Err(format!("could not check the bead against the base metal: {refusal}")),
    }
}

/// Check the built bead exactly: it shares no volume with any resident body.
pub(super) fn clear(scene: &SceneMap, bead: &crate::BrepSolid, exclude: &str, tolerance: f64) -> Result<Result<(), NoFit>, String> {
    let (low, high) = bounds(bead);
    let mut names: Vec<(&String, &u32)> = scene.solids.iter().filter(|(name, _)| !owned_by(name, exclude)).collect();
    names.sort();
    for (name, &handle) in names {
        let body = crate::registered_solid_clone(handle)?;
        let (body_low, body_high) = bounds(&body);
        let apart = body_low.x > high.x + tolerance || body_low.y > high.y + tolerance || body_low.z > high.z + tolerance
            || body_high.x < low.x - tolerance || body_high.y < low.y - tolerance || body_high.z < low.z - tolerance;
        if apart {
            continue;
        }
        match common_volume(bead, &body) {
            Ok(shared) if shared > LEFTOVER_VOLUME => return Ok(Err(format!("the bead would overlap {name}"))),
            Ok(_) => {}
            Err(reason) => return Ok(Err(reason)),
        }
    }
    Ok(Ok(()))
}

/// [`clear`] with some bodies as the weld leaves them: each `(name, body)`
/// in `replaced` stands for the resident body of that name (a part the weld
/// prepares), and the weld's own beads are skipped.
pub(super) fn clear_of(scene: &SceneMap, bead: &crate::BrepSolid, exclude: &str, replaced: &[(&str, &crate::BrepSolid)], tolerance: f64) -> Result<Result<(), NoFit>, String> {
    let (low, high) = bounds(bead);
    let mut names: Vec<(&String, &u32)> = scene.solids.iter().filter(|(name, _)| !owned_by(name, exclude)).collect();
    names.sort();
    for (name, &handle) in names {
        let body = match replaced.iter().find(|(replaced, _)| *replaced == name.as_str()) {
            Some((_, body)) => (*body).clone(),
            None => crate::registered_solid_clone(handle)?,
        };
        let (body_low, body_high) = bounds(&body);
        let apart = body_low.x > high.x + tolerance || body_low.y > high.y + tolerance || body_low.z > high.z + tolerance
            || body_high.x < low.x - tolerance || body_high.y < low.y - tolerance || body_high.z < low.z - tolerance;
        if apart {
            continue;
        }
        match common_volume(bead, &body) {
            Ok(shared) if shared > LEFTOVER_VOLUME => return Ok(Err(format!("the bead would overlap {name}"))),
            Ok(_) => {}
            Err(reason) => return Ok(Err(reason)),
        }
    }
    Ok(Ok(()))
}

/// Whether `body` backs the thin `slab` under a contact: nothing of the slab
/// is left once the body is removed.
pub(super) fn backed(slab: &crate::BrepSolid, body: &crate::BrepSolid) -> Result<Result<(), NoFit>, String> {
    match boolean(slab, body, crate::BooleanOperation::Subtract) {
        Err(refusal) if empty(&refusal) => Ok(Ok(())),
        Ok(rest) => {
            let left = crate::solid_mass_properties(&rest).map(|m| m.volume.abs()).unwrap_or(f64::INFINITY);
            Ok(if left > LEFTOVER_VOLUME { Err(format!("{left} of the support slab lies outside the part")) } else { Ok(()) })
        }
        Err(refusal) => Ok(Err(format!("could not check the support: {refusal}"))),
    }
}

/// Check each leg's whole strip on its joint face is backed by that face's
/// body (a thin slab under the strip, carried along the seam as the bead is
/// and trimmed as it is, lies entirely inside the body), so a bead never
/// bridges a hole, an edge or a gap in the face.
pub(super) fn supported(seam: &Seam, joint: &Fit, legs: [f64; 2], trim: &Trim) -> Result<Result<(), NoFit>, String> {
    for (i, (face, (direction, size))) in joint.faces.iter().zip(joint.legs.iter().zip(legs)).enumerate() {
        // Shallow enough for sheet-thin joint faces: the support check needs
        // only this much material under the leg.
        let depth = support_depth(seam);
        let start = seam.start.add(joint.root);
        let toe = start.add(direction.scale(size));
        // A leg across a root gap is backed from where its face begins.
        let start = start.add(direction.scale(joint.skip[i]));
        let inward = face.normal.scale(-depth);
        // Where the faces meet at other than a right angle, the slab's inner
        // corner at the seam would reach past the other joint face's plane (out
        // of this body, a cone on a plate); it is moved along the leg onto that
        // plane.
        let other = joint.faces[1 - i].normal;
        let shift = (depth * face.normal.dot(other) / direction.dot(other)).max(0.0);
        let slab = sweep(seam, &[start, toe, toe.add(inward), start.add(direction.scale(shift)).add(inward)], trim)?;
        let body = crate::registered_solid_clone(face.handle)?;
        // Backed means nothing of the slab is left once the body is removed:
        // any hole, edge or gap under the leg leaves a piece.
        match boolean(&slab, &body, crate::BooleanOperation::Subtract) {
            Err(refusal) if empty(&refusal) => {}
            Ok(rest) => {
                let left = crate::solid_mass_properties(&rest).map(|m| m.volume.abs()).unwrap_or(f64::INFINITY);
                if left > LEFTOVER_VOLUME {
                    return Ok(Err(format!("leg {size} is not supported along its whole length by joint face {}", face.name)));
                }
            }
            Err(refusal) => return Ok(Err(format!("could not check the support under leg {size}: {refusal}"))),
        }
    }
    Ok(Ok(()))
}

/// The material depth the support check reads under each leg: a hundred weld
/// floors (0.001 model units), ten times the band under which the boolean
/// reads two parallel planes as coincident, so the slab's floor is always
/// resolvable. Only a joint body thinner than this is refused as unsupported.
pub(super) fn support_depth(_seam: &Seam) -> f64 {
    100.0 * crate::tolerance::WELD_FLOOR
}

/// The seam two picked faces meet along: the one straight or circular edge of either face
/// that the other face carries (a concave edge both faces bound, or a plate's
/// edge standing on the other plate's face). Returned by its scene edge name.
pub(super) fn seam_between(scene: &SceneMap, faces: [&str; 2]) -> Result<Result<String, String>, String> {
    let mut found: Vec<(String, Vec3, Vec3, Vec3)> = Vec::new();
    for (own, other) in [(faces[0], faces[1]), (faces[1], faces[0])] {
        for face in scene.resolve_face_group(own) {
            let runs: Vec<(u64, NurbsCurve)> = crate::with_registered_solid_str(face.handle, |solid| {
                let record = solid.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face.face_id)
                    .ok_or_else(|| format!("face '{own}' is no longer resident"))?;
                let mut runs = Vec::new();
                for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
                    let Some(edge) = solid.edges.iter().find(|e| e.id == coedge.edge_id) else { continue };
                    if edge.degenerate { continue; }
                    runs.push((edge.id, crate::feature_pipeline::features::common::trimmed_curve(&edge.curve, edge.t0, edge.t1)?));
                }
                Ok(runs)
            })?;
            for (edge_id, curve) in runs {
                let Ok(Some(seam)) = Seam::from_curve(&curve, None) else { continue };
                if seam.length <= seam.tolerance() { continue; }
                let (start, end, middle) = (seam.at(0.0), seam.at(1.0), seam.at(0.5));
                if !carries(scene, &seam, other)? { continue; }
                let mut names: Vec<&String> = scene.edges.iter()
                    .filter(|(_, edge)| edge.handle == face.handle && edge.edge_id == edge_id)
                    .map(|(name, _)| name).collect();
                names.sort();
                let Some(name) = names.first() else { continue };
                let tolerance = seam.tolerance();
                // The same edge seen from both faces: same ends and middle (two
                // half circles share their ends, not their middles).
                let same = |(_, a, b, m): &(String, Vec3, Vec3, Vec3)| m.sub(middle).length() < tolerance
                    && ((a.sub(start).length() < tolerance && b.sub(end).length() < tolerance)
                        || (a.sub(end).length() < tolerance && b.sub(start).length() < tolerance));
                if !found.iter().any(same) {
                    found.push(((*name).clone(), start, end, middle));
                }
            }
        }
    }
    Ok(match found.as_slice() {
        [(name, _, _, _)] => Ok(name.clone()),
        [] => Err(format!("faces '{}' and '{}' do not meet along a straight or circular edge; select the seam edge", faces[0], faces[1])),
        _ => Err(format!("faces '{}' and '{}' meet along several edges; select the seam edge", faces[0], faces[1])),
    })
}
