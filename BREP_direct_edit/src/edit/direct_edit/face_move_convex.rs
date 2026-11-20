//! The CONVEX re-cut: a convex body whose every face is a plane, a cylinder
//! or a single-nappe cone, re-made with its selected planar faces carried
//! rigidly — as the intersection of every face's own material region.
//!
//! A convex body is the intersection of its faces' material regions: the
//! half-space behind each plane, the solid inside each cylinder or cone. A
//! planar face carried rigidly is still a plane, so the body the motion
//! defines is the same intersection with that plane's half-space carried. It
//! is the answer whichever faces the moved plane now crosses — the case the
//! rotation road cannot reach is a whole frustum's top turned past the
//! generatrices: its closed rim would reach the generatrices parallel to the
//! plane, which never end on it, and the body that exists has the turned top
//! cutting the BASE, an edge the rotation road does not mint.
//!
//! It is offered only after the rotation road refused the section, and it
//! reads nothing of that road's body. Each step declines by name:
//!
//! * every selected face is a plane, every other face a plane, a cylinder or
//!   a cone bounding material on its inside (`not_convex`, `curved_neighbour`),
//!   and no unselected face lies on a selected face's plane
//!   (`coplanar_unselected`: the road moves the whole plane);
//! * the IDENTITY: the same intersection with the selection UNMOVED is the
//!   input body to the volume tolerance, in one shell. That is what makes the
//!   body convex in the sense used here; an L-shaped part fails it
//!   (`not_convex`);
//! * the result is one non-empty shell clear of every artificial bound. A
//!   result reaching the cube every region is clipped to is re-made in one ten
//!   times larger, up to a thousand times, and one reaching the cap a cone's
//!   tool keeps short of its apex is re-made with the whole nappe; a result
//!   still on the largest cube is refused (`beyond_reach`: a cylinder's cap
//!   turned parallel to its axis has no body; a wedge reaching farther than a
//!   thousand times the body's size is not built either).
//!
//! Names come back by carrier, faces and edges, as the re-drill's do.

use super::face_move_recut::{convex_solid, three_plane_point, HalfSpace};
use super::*;
use crate::offset_retrim::plane_of_surface;
use crate::{boolean_operation, make_cone_brep, make_cylinder_brep, make_sphere_brep, make_torus_brep, BooleanOperation, BooleanOptions};
use crate::{KernelRefusal, KernelStage, OrRefuse};

const OP: &str = "transformFace(convex re-cut)";
const VOLUME_TOLERANCE: f64 = 1e-9;

fn declined(what: &str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP}: {message}"))
}

/// A ruled revolution's carrier: radius `rho0 + (rho1 - rho0)·z / height` at
/// axial station `z` from `origin` along `axis`.
#[derive(Clone, Copy, Debug)]
struct Ruled {
    origin: Vec3,
    axis: Vec3,
    rho0: f64,
    slope: f64,
}

impl Ruled {
    fn radius(&self, z: f64) -> f64 {
        self.rho0 + self.slope * z
    }

    fn station(&self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.axis)
    }

    /// The same point set: parallel axes on one line, the same radius at the
    /// same station, the same slope.
    fn same(&self, other: &Ruled, band: f64) -> bool {
        let sign = if self.axis.dot(other.axis) < 0.0 { -1.0 } else { 1.0 };
        if self.axis.cross(other.axis).length() > 1e-9 {
            return false;
        }
        let offset = other.origin.sub(self.origin);
        if offset.sub(self.axis.scale(offset.dot(self.axis))).length() > band {
            return false;
        }
        let z = self.station(other.origin);
        (self.radius(z) - other.rho0).abs() <= band && (self.slope - sign * other.slope).abs() <= 1e-9
    }
}

fn ruled_carrier(face: &FaceRecord) -> Option<Ruled> {
    match face.surface.analytic()? {
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } if *height > 0.0 => Some(Ruled {
            origin: frame.origin,
            axis: frame.axis,
            rho0: *rho0,
            slope: (rho1 - rho0) / height,
        }),
        _ => None,
    }
}

/// A point on the face's carrier at its parameter-domain centre, and its
/// outward normal there.
fn sample(face: &FaceRecord) -> Option<(Vec3, Vec3)> {
    let [u0, u1] = face.surface.domain_u().ok()?;
    let [v0, v1] = face.surface.domain_v().ok()?;
    let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let point = face.surface.evaluate(u, v).ok()?;
    let normal = face.surface.normal(u, v).ok()?;
    Some((point, if face.same_sense { normal } else { normal.scale(-1.0) }))
}

/// One face's material region, before the clip.
enum Region {
    Half(HalfSpace),
    Inside(Ruled, f64),
    Closed(Closed),
}

/// A closed carrier bounding material on its inside: the solid ball or the
/// solid torus. Its region needs no clip.
#[derive(Clone, Copy, Debug)]
enum Closed {
    Ball { centre: Vec3, radius: f64 },
    Torus { centre: Vec3, axis: Vec3, major: f64, minor: f64 },
}

impl Closed {
    fn of(face: &FaceRecord) -> Option<Closed> {
        match face.surface.analytic()? {
            AnalyticSurface::Sphere { frame, radius } => Some(Closed::Ball { centre: frame.origin, radius: *radius }),
            AnalyticSurface::Torus { frame, major_radius, minor_radius } => Some(Closed::Torus {
                centre: frame.origin,
                axis: frame.axis,
                major: *major_radius,
                minor: *minor_radius,
            }),
            _ => None,
        }
    }

    /// The point the material at `point` surrounds: the ball's centre, or the
    /// torus's tube centre nearest `point`.
    fn core(&self, point: Vec3) -> Option<Vec3> {
        match *self {
            Closed::Ball { centre, .. } => Some(centre),
            Closed::Torus { centre, axis, major, .. } => {
                let offset = point.sub(centre);
                let radial = offset.sub(axis.scale(offset.dot(axis))).normalized().ok()?;
                Some(centre.add(radial.scale(major)))
            }
        }
    }

    fn same(&self, other: &Closed, band: f64) -> bool {
        match (*self, *other) {
            (Closed::Ball { centre: a, radius: r }, Closed::Ball { centre: b, radius: s }) => {
                a.sub(b).length() <= band && (r - s).abs() <= band
            }
            (
                Closed::Torus { centre: a, axis: p, major: r, minor: m },
                Closed::Torus { centre: b, axis: q, major: s, minor: n },
            ) => a.sub(b).length() <= band && p.cross(q).length() <= 1e-9 && (r - s).abs() <= band && (m - n).abs() <= band,
            _ => false,
        }
    }

    fn tool(&self) -> Result<BrepSolid, String> {
        match *self {
            Closed::Ball { centre, radius } => make_sphere_brep(centre, radius, Vec3::new(0.0, 0.0, 1.0)),
            Closed::Torus { centre, axis, major, minor } => make_torus_brep(centre, axis, major, minor),
        }
    }
}

fn oriented_plane(face: &FaceRecord, band: f64) -> Option<HalfSpace> {
    let plane = plane_of_surface(&face.surface, band, OP).ok()?;
    let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
    Some(HalfSpace { normal, offset: normal.dot(plane.origin) })
}

fn regions(solid: &BrepSolid, band: f64, excluded: &[u64]) -> Result<Vec<(u64, Option<String>, Region)>, KernelRefusal> {
    let mut out = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces).filter(|face| !excluded.contains(&face.id)) {
        if let Some(space) = oriented_plane(face, band) {
            out.push((face.id, face.name.clone(), Region::Half(space)));
            continue;
        }
        if let Some(closed) = Closed::of(face) {
            let (point, normal) = sample(face)
                .ok_or_else(|| declined("curved_neighbour", format!("face {} cannot be sampled", face.id)))?;
            let core = closed
                .core(point)
                .ok_or_else(|| declined("curved_neighbour", format!("face {} sits on its torus's axis", face.id)))?;
            if normal.dot(point.sub(core)) <= 0.0 {
                return Err(declined("not_convex", format!("face {} bounds a hollow, not material", face.id)));
            }
            out.push((face.id, face.name.clone(), Region::Closed(closed)));
            continue;
        }
        let Some(ruled) = ruled_carrier(face) else {
            return Err(declined(
                "curved_neighbour",
                format!("face {} is neither a plane, a cylinder, a cone, a sphere nor a torus", face.id),
            ));
        };
        let (point, normal) = sample(face)
            .ok_or_else(|| declined("curved_neighbour", format!("face {} cannot be sampled", face.id)))?;
        let offset = point.sub(ruled.origin);
        let radial = offset.sub(ruled.axis.scale(offset.dot(ruled.axis)));
        if normal.dot(radial) <= 0.0 {
            return Err(declined("not_convex", format!("face {} bounds a hole, not material", face.id)));
        }
        let station = ruled.station(point);
        out.push((face.id, face.name.clone(), Region::Inside(ruled, station)));
    }
    Ok(out)
}

/// The rigid motion, as a point map and a direction map.
struct Motion {
    rotation: Option<AffineTransform>,
    translation: Vec3,
}

impl Motion {
    fn point(&self, point: Vec3) -> Vec3 {
        match &self.rotation {
            None => point,
            Some(rotation) => rotation.point(point),
        }
        .add(self.translation)
    }

    fn half_space(&self, space: HalfSpace) -> Result<HalfSpace, KernelRefusal> {
        let (normal, point) = match &self.rotation {
            None => (space.normal, space.normal.scale(space.offset)),
            Some(rotation) => (
                super::face_rotate::rotate_direction(rotation, space.normal)
                    .normalized()
                    .or_refuse(KernelStage::Collect, "rotation")?,
                rotation.point(space.normal.scale(space.offset)),
            ),
        };
        let point = point.add(self.translation);
        Ok(HalfSpace { normal, offset: normal.dot(point) })
    }
}

/// The intersection of the regions, the selected planes carried by `motion`
/// (or not, for the identity), clipped to the cube `(centre, half)`. Returns
/// the body and the artificial planes it must not touch.
fn intersect(
    regions: &[(u64, Option<String>, Region)],
    selected: &[u64],
    motion: Option<&Motion>,
    (centre, half): (Vec3, f64),
    apex_whole: bool,
    tolerance: f64,
) -> Result<(BrepSolid, Vec<HalfSpace>, Vec<HalfSpace>), KernelRefusal> {
    let mut artificial: Vec<HalfSpace> = Vec::new();
    let mut caps: Vec<HalfSpace> = Vec::new();
    for axis in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)] {
        for sign in [1.0, -1.0] {
            let normal = axis.scale(sign);
            artificial.push(HalfSpace { normal, offset: normal.dot(centre) + half });
        }
    }
    let mut spaces = artificial.clone();
    for (id, _, region) in regions {
        if let Region::Half(space) = region {
            let space = match motion {
                Some(motion) if selected.contains(id) => motion.half_space(*space)?,
                _ => *space,
            };
            // A plane split into several coplanar faces is one half-space.
            let band = tolerance.max(1e-9) * 1e3;
            if !spaces.iter().any(|other| other.normal.sub(space.normal).length() <= 1e-9 && (other.offset - space.offset).abs() <= band) {
                spaces.push(space);
            }
        }
    }
    // An inside point: the mean of the polytope's corners.
    let band = tolerance.max(1e-9) * 10.0;
    let mut corners: Vec<Vec3> = Vec::new();
    for a in 0..spaces.len() {
        for b in (a + 1)..spaces.len() {
            for c in (b + 1)..spaces.len() {
                let Some(point) = three_plane_point(&spaces[a], &spaces[b], &spaces[c]) else { continue };
                if spaces.iter().all(|space| space.normal.dot(point) - space.offset <= band) {
                    corners.push(point);
                }
            }
        }
    }
    if corners.len() < 4 {
        return Err(declined("unbounded", "the planes enclose no volume".into()));
    }
    let inside = corners.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, point| sum.add(*point)).scale(1.0 / corners.len() as f64);
    let mut body = convex_solid(&spaces, inside, tolerance)
        .map_err(|error| declined("unbounded", format!("the planes' polytope does not build: {error}")))?;

    // Where the moved plane leans, the material it takes is on its outward
    // side: a tool's SEAM put there stays off the faces the result keeps, so
    // the boolean does not hand a wall back in two pieces.
    let seam_hint = motion.and_then(|motion| {
        regions.iter().find_map(|(id, _, region)| match region {
            Region::Half(space) if selected.contains(id) => motion.half_space(*space).ok().map(|space| space.normal),
            _ => None,
        })
    });
    for (_, _, region) in regions {
        if let Region::Closed(closed) = region {
            let tool = closed.tool().map_err(|error| declined("tool", format!("the region's tool does not build: {error}")))?;
            body = boolean_operation(&body, &tool, BooleanOperation::Intersect, &BooleanOptions::default())
                .map_err(|error| error.with_message(|message| format!("{OP}: the intersection refused — {message}")))?;
            continue;
        }
        let Region::Inside(ruled, station) = region else { continue };
        // The cube's extent along this axis.
        let reach = half * 3f64.sqrt();
        let middle = ruled.station(centre);
        let (mut from, mut to) = (middle - reach, middle + reach);
        // A cone's tool is its face's own nappe. Kept a little short of the
        // apex, the short end is an artificial cap the body must not reach;
        // a body that does reach it is re-made with the whole nappe, apex
        // included (`apex_whole`).
        let mut apex_end = None;
        if ruled.slope.abs() > 1e-12 {
            let apex = -ruled.rho0 / ruled.slope;
            let keep_short = if apex_whole { 0.0 } else { 1e-3 * half };
            if *station > apex {
                if apex + keep_short > from {
                    from = apex + keep_short;
                    apex_end = Some(from);
                }
            } else if apex - keep_short < to {
                to = apex - keep_short;
                apex_end = Some(to);
            }
            if let (Some(end), false) = (apex_end, apex_whole) {
                let normal = if *station > apex { ruled.axis.scale(-1.0) } else { ruled.axis };
                caps.push(HalfSpace { normal, offset: normal.dot(ruled.origin.add(ruled.axis.scale(end))) });
            }
        }
        if to - from <= 0.0 {
            return Err(declined("unbounded", "a cone's nappe misses the clip".into()));
        }
        // Built from its wide end, so an apex end is the tool's zero-radius top.
        let (wide, narrow) = if ruled.radius(from).abs() >= ruled.radius(to).abs() { (from, to) } else { (to, from) };
        let direction = if wide <= narrow { ruled.axis } else { ruled.axis.scale(-1.0) };
        let base = ruled.origin.add(ruled.axis.scale(wide));
        let narrow_radius = if apex_whole && apex_end.is_some() { 0.0 } else { ruled.radius(narrow).abs() };
        let tool = if ruled.slope.abs() <= 1e-12 {
            make_cylinder_brep(base, direction, ruled.rho0.abs(), (to - from).abs())
        } else {
            make_cone_brep(base, direction, ruled.radius(wide).abs(), narrow_radius, (to - from).abs())
        }
        .map_err(|error| declined("tool", format!("the region's tool does not build: {error}")))?;
        let tool = match seam_hint {
            Some(hint) => turn_seam_towards(tool, ruled, hint)?,
            None => tool,
        };
        body = boolean_operation(&body, &tool, BooleanOperation::Intersect, &BooleanOptions::default())
            .map_err(|error| error.with_message(|message| format!("{OP}: the intersection refused — {message}")))?;
    }
    Ok((body, artificial, caps))
}

/// The tool turned about its own axis so its seam generatrix lies on the
/// side `hint` points to, perpendicular to the axis; unchanged where `hint`
/// runs along the axis.
fn turn_seam_towards(tool: BrepSolid, ruled: &Ruled, hint: Vec3) -> Result<BrepSolid, KernelRefusal> {
    let wanted = hint.sub(ruled.axis.scale(hint.dot(ruled.axis)));
    if wanted.length() <= 0.1 {
        return Ok(tool);
    }
    let Some(seam) = tool.shells.iter().flat_map(|shell| &shell.faces).find_map(|face| match face.surface.analytic() {
        Some(AnalyticSurface::RuledRevolution { frame, .. }) => Some(frame.x_axis),
        _ => None,
    }) else {
        return Ok(tool);
    };
    let angle = seam.cross(wanted).dot(ruled.axis).atan2(seam.dot(wanted));
    let rotation = super::face_rotate::rotation_about(ruled.origin, ruled.axis, angle).or_refuse(KernelStage::Collect, "rotation")?;
    crate::transform_brep(&tool, rotation, false).or_refuse(KernelStage::Collect, "transform_brep")
}

/// The intersection, grown until it is clear of every artificial bound. A
/// body reaching the clip cube is re-made in a cube ten times larger, up to a
/// thousand times; one reaching a cone tool's short cap is re-made with the
/// whole nappe to its apex. A body still on the cube at the largest is
/// refused as `beyond_reach`: unbounded there, or reaching farther than a
/// thousand times the body's size — this road does not tell the two apart.
fn solve(
    regions: &[(u64, Option<String>, Region)],
    selected: &[u64],
    motion: Option<&Motion>,
    (centre, half): (Vec3, f64),
    band: f64,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let touches = |body: &BrepSolid, planes: &[HalfSpace], size: f64| {
        let touch = band.max(1e-7 * size);
        body.vertices
            .iter()
            .map(|vertex| vertex.point)
            .find(|point| planes.iter().any(|space| (space.normal.dot(*point) - space.offset).abs() <= touch))
    };
    let mut last = None;
    for grow in [1.0, 10.0, 100.0, 1000.0] {
        let size = half * grow;
        let (mut body, mut cube, caps) = intersect(regions, selected, motion, (centre, size), false, tolerance)?;
        if touches(&body, &caps, size).is_some() {
            (body, cube, _) = intersect(regions, selected, motion, (centre, size), true, tolerance)?;
        }
        match touches(&body, &cube, size) {
            None => return Ok(body),
            Some(point) => last = Some(point),
        }
    }
    Err(declined(
        "beyond_reach",
        format!(
            "the moved body still reaches the clip at {:?} with the clip a thousand times the \
             body's size: the faces left open there either have no body at all or one reaching \
             farther than this road builds",
            last.unwrap_or_default()
        ),
    ))
}

/// Re-make a convex body with its selected PLANAR faces carried by the rigid
/// motion `turn` (axis point, unit axis, angle in radians; applied first) then
/// `translation`. See the module comment for what it checks.
pub fn recut_convex_body(
    solid: &BrepSolid,
    face_ids: &[u64],
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = solid_model_scale(solid).max(1.0) * 1e-9;
    let band = tolerance * 1e3;
    if solid.shells.len() != 1 {
        return Err(declined("not_convex", "the body has more than one shell".into()));
    }
    let regions = regions(solid, band, &[])?;
    for id in face_ids {
        let Some((_, _, Region::Half(space))) = regions.iter().find(|(face, ..)| face == id) else {
            return Err(declined("curved_carrier", format!("selected face {id} is not a plane")));
        };
        // The road moves the selected face's whole half-space, so a face it
        // did not select on the same plane — the other end disk of a torus
        // cut through both arms — would be carried with it.
        if let Some((other, ..)) = regions.iter().find(|(other, _, region)| {
            !face_ids.contains(other)
                && matches!(region, Region::Half(o) if o.normal.sub(space.normal).length() <= 1e-9 && (o.offset - space.offset).abs() <= band)
        }) {
            return Err(declined(
                "coplanar_unselected",
                format!("face {other} lies on selected face {id}'s plane but is not selected; moving the plane would move it too"),
            ));
        }
    }
    let motion = Motion {
        rotation: match turn {
            None => None,
            Some((point, axis, angle)) => Some(
                super::face_rotate::rotation_about(point, axis, angle).or_refuse(KernelStage::Collect, "rotation")?,
            ),
        },
        translation,
    };
    let bounds = crate::recentre::solid_bounds(solid);
    let centre = bounds.minimum.add(bounds.maximum).scale(0.5);
    let diagonal = bounds.maximum.sub(bounds.minimum).length();
    if !diagonal.is_finite() {
        return Err(declined("unbounded", "the body's bounds cannot be read".into()));
    }
    let pivot_reach = turn.map(|(point, ..)| point.sub(centre).length()).unwrap_or(0.0);
    let half = 2.0 * (diagonal + pivot_reach) + translation.length();
    let input_volume = solid_signed_volume(solid).map(f64::abs).or_refuse(KernelStage::Collect, "volume")?;

    let identity = solve(&regions, face_ids, None, (centre, half), band, tolerance)?;
    let identity_volume = solid_signed_volume(&identity).map(f64::abs).or_refuse(KernelStage::Collect, "volume")?;
    if identity.shells.len() != 1 || (identity_volume - input_volume).abs() > VOLUME_TOLERANCE * input_volume {
        return Err(declined(
            "not_convex",
            format!(
                "the intersection of the faces' own regions measures {identity_volume} in {} \
                 shell(s), not the body's {input_volume}: the body is not that intersection",
                identity.shells.len()
            ),
        ));
    }

    let body = solve(&regions, face_ids, Some(&motion), (centre, half), band, tolerance)?;
    let volume = solid_signed_volume(&body).map(f64::abs).or_refuse(KernelStage::Collect, "volume")?;
    if body.shells.len() != 1 || volume <= VOLUME_TOLERANCE * input_volume {
        return Err(declined(
            "unbounded",
            format!("the moved intersection is {} shell(s) of volume {volume}", body.shells.len()),
        ));
    }
    // The intersections can hand a carrier back in pieces (a cone wall split
    // along its seam); one carrier is one face, as the input had it.
    let body = crate::merge_same_surface_faces(&body, tolerance).unwrap_or(body);
    let mut body = restamp(body, solid, &regions, face_ids, &motion, band);
    super::face_move_redrill::restore_edge_names(&mut body, solid, band);
    Ok(body)
}

/// The body with the faces `excluded` taken away and filled in from the
/// others: the intersection of every other face's own region. What it fills is
/// not checked here — the caller's identity check reads it (the re-drill: the
/// filled body less the bore's own carrier must be the input).
pub(super) fn filled_without(solid: &BrepSolid, excluded: &[u64]) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = solid_model_scale(solid).max(1.0) * 1e-9;
    let band = tolerance * 1e3;
    let regions = regions(solid, band, excluded)?;
    let bounds = crate::recentre::solid_bounds(solid);
    let centre = bounds.minimum.add(bounds.maximum).scale(0.5);
    let diagonal = bounds.maximum.sub(bounds.minimum).length();
    if !diagonal.is_finite() {
        return Err(declined("unbounded", "the body's bounds cannot be read".into()));
    }
    let body = solve(&regions, &[], None, (centre, 2.0 * diagonal), band, tolerance)?;
    let body = crate::merge_same_surface_faces(&body, tolerance).unwrap_or(body);
    Ok(restamp(body, solid, &regions, &[], &Motion { rotation: None, translation: Vec3::new(0.0, 0.0, 0.0) }, band))
}

/// The input's names back, by carrier: a plane (moved, for the selection) by
/// its plane, a cylinder or cone by its carrier; a second face on one carrier
/// takes `name_1`.
fn restamp(
    mut body: BrepSolid,
    solid: &BrepSolid,
    regions: &[(u64, Option<String>, Region)],
    selected: &[u64],
    motion: &Motion,
    band: f64,
) -> BrepSolid {
    // Each name with its carrier and a point of its face (moved, for the
    // selection): faces sharing one carrier — two coplanar end disks — are told
    // apart by which original face a result face is nearest.
    let mut wanted: Vec<(String, Region, Option<Vec3>)> = Vec::new();
    for (id, name, region) in regions {
        let Some(name) = name.clone() else { continue };
        let at = solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == *id).and_then(|face| face_point(solid, face));
        match region {
            Region::Half(space) if selected.contains(id) => {
                if let Ok(moved) = motion.half_space(*space) {
                    wanted.push((name, Region::Half(moved), at.map(|point| motion.point(point))));
                }
            }
            Region::Half(space) => wanted.push((name, Region::Half(*space), at)),
            Region::Inside(ruled, station) => wanted.push((name, Region::Inside(*ruled, *station), at)),
            Region::Closed(closed) => wanted.push((name, Region::Closed(*closed), at)),
        }
    }
    let points: HashMap<u64, Vec3> = body
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter_map(|face| face_point(&body, face).map(|point| (face.id, point)))
        .collect();
    let mut used: HashMap<String, usize> = HashMap::default();
    for shell in &mut body.shells {
        for face in &mut shell.faces {
            let on_carrier = |region: &Region| {
                if let Some(space) = oriented_plane(face, band) {
                    matches!(region, Region::Half(other)
                        if other.normal.sub(space.normal).length() <= 1e-9 && (other.offset - space.offset).abs() <= band)
                } else if let Some(closed) = Closed::of(face) {
                    matches!(region, Region::Closed(other) if other.same(&closed, band))
                } else if let Some(ruled) = ruled_carrier(face) {
                    matches!(region, Region::Inside(other, _) if other.same(&ruled, band))
                } else {
                    false
                }
            };
            let here = points.get(&face.id).copied();
            let distance = |at: &Option<Vec3>| match (here, at) {
                (Some(here), Some(at)) => here.sub(*at).length(),
                _ => f64::INFINITY,
            };
            let found = wanted
                .iter()
                .filter(|(_, region, _)| on_carrier(region))
                .min_by(|a, b| distance(&a.2).total_cmp(&distance(&b.2)));
            let Some((name, ..)) = found else {
                face.name = None;
                continue;
            };
            let count = used.entry(name.clone()).or_insert(0);
            *count += 1;
            face.name = Some(if *count == 1 { name.clone() } else { format!("{name}_{}", *count - 1) });
        }
    }
    body
}

/// The mean of a face's boundary vertices.
fn face_point(solid: &BrepSolid, face: &FaceRecord) -> Option<Vec3> {
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let mut count = 0usize;
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let edge = solid.edges.iter().find(|edge| edge.id == coedge.edge_id)?;
        for id in [edge.start_vertex_id, edge.end_vertex_id] {
            sum = sum.add(solid.vertices.iter().find(|vertex| vertex.id == id)?.point);
            count += 1;
        }
    }
    (count > 0).then(|| sum.scale(1.0 / count as f64))
}
