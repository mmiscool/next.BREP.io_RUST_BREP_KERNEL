//! Flare grooves: the groove a round face makes against another round face
//! (flare-V: two round bars, two bent edges) or against a flat face
//! (flare-bevel: a round bar or a bent edge against a plate).
//!
//! The round faces are cylinders with parallel axes, parallel to the flat
//! face. In the section square to the axes, `x` runs across the joint from
//! the first face's root (the point nearest the other face) to the second's,
//! `y` from the axes' level towards the side the groove opens to. The fill
//! is bounded by each face's arc from its root up to the flare's top (the
//! smaller round's tangent level: a quarter of it), a straight line across
//! the top (or a cap bulging past it), and, where the faces stand a root
//! opening apart, a straight line across the opening at the root. A groove
//! depth less than the full flare raises the bottom line instead.
//!
//! Checks, as every groove: the fill is clear of every body (no common
//! volume) and a slab just inside each part backs every side it lies
//! against (a concentric arc shell under a round face).
use super::common;
use super::fit::{self, Seam, Side, Trim};
use super::{arc_flat, slab, slab_depth, Flat, Region};
use crate::feature_pipeline::{FeatureContext, FeatureResult, ScenePoint};
use crate::Vec3;

/// Which flare.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Flare {
    /// Two round faces.
    V,
    /// A flat face (x ≤ 0 in the section) and a round one.
    Bevel,
}

/// One picked face as a weld reads it.
#[derive(Clone, Debug)]
pub(super) enum Shape {
    /// A plane through `on`, outward `normal`.
    Flat { on: Vec3, normal: Vec3 },
    /// A circular cylinder: `centre` on the axis, unit `axis`, `radius`;
    /// `convex` when the part lies inside it (a bar), else outside (a bore).
    Round { centre: Vec3, axis: Vec3, radius: f64, convex: bool },
}

pub(super) struct Face {
    pub name: String,
    pub solid: String,
    pub handle: u32,
    face_id: u64,
    pub shape: Shape,
    /// Points of the face's boundary (to measure its run along the seam).
    pub boundary: Vec<Vec3>,
    /// The face's straight boundary edges, end to end.
    pub lines: Vec<(Vec3, Vec3)>,
}

/// Read a picked face: a plane, or a circular cylinder (exact to 1e-9
/// relative at a grid of surface points).
pub(super) fn face(ctx: &FeatureContext, name: &str) -> Result<Face, String> {
    let group = ctx.scene.resolve_face_group(name);
    let [picked] = group.as_slice() else { return Err(format!("face '{name}' stands for several faces; select one")) };
    let solid = ctx.scene.solids.iter().filter(|(_, handle)| **handle == picked.handle)
        .map(|(name, _)| name.clone()).min().ok_or_else(|| format!("face '{name}' belongs to no resident part"))?;
    let (shape, boundary, lines) = crate::with_registered_solid_str(picked.handle, |body| {
        let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == picked.face_id)
            .ok_or_else(|| format!("face '{name}' is no longer resident"))?;
        let mut boundary = Vec::new();
        let mut lines = Vec::new();
        for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
            let Some(edge) = body.edges.iter().find(|e| e.id == coedge.edge_id) else { continue };
            if edge.curve.degree == 1 && !edge.degenerate {
                lines.push((edge.curve.evaluate(edge.t0)?, edge.curve.evaluate(edge.t1)?));
            }
            for k in 0..=32 {
                boundary.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 32.0)?);
            }
        }
        // The carrier at a grid of points: positions and outward normals.
        let [u0, u1] = record.surface.domain_u()?;
        let [v0, v1] = record.surface.domain_v()?;
        let mut samples = Vec::new();
        for i in 0..=6 {
            for j in 0..=6 {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / 6.0, v0 + (v1 - v0) * j as f64 / 6.0);
                let normal = record.surface.normal(u, v)?;
                let outward = if record.same_sense { normal } else { normal.scale(-1.0) };
                samples.push((record.surface.evaluate(u, v)?, outward.normalized()?));
            }
        }
        let size = samples.iter().map(|(p, _)| p.sub(samples[0].0).length()).fold(1.0, f64::max);
        let tolerance = 1e-9 * size;
        let (on, normal) = samples[0];
        if samples.iter().all(|(p, n)| p.sub(on).dot(normal).abs() <= tolerance && n.sub(normal).length() <= 1e-9) {
            return Ok((Shape::Flat { on, normal }, boundary, lines));
        }
        // A cylinder: every normal square to one axis, every point one
        // radius from it, the normal pointing away from it (convex).
        let not_round = || format!("face '{name}' is neither flat nor a round (cylindrical) face; a flare groove joins round faces, or a round face to a flat one");
        let (mut best, mut pair) = (0.0, (0, 0));
        for (i, (_, a)) in samples.iter().enumerate() {
            for (j, (_, b)) in samples.iter().enumerate().skip(i + 1) {
                let cross = a.cross(*b).length();
                if cross > best { best = cross; pair = (i, j); }
            }
        }
        if best < 1e-3 { return Err(not_round()); }
        let ((pa, na), (pb, nb)) = (samples[pair.0], samples[pair.1]);
        let axis = na.cross(nb).normalized()?;
        // pa + s·na and pb + t·nb meet on the axis.
        let s = pb.sub(pa).cross(nb).dot(axis) / na.cross(nb).dot(axis);
        let centre = pa.add(na.scale(s));
        // The outward normal points away from the axis on a bar, towards it
        // in a bore.
        let (radius, outward) = (s.abs(), if s < 0.0 { 1.0 } else { -1.0 });
        if radius <= tolerance {
            return Err(not_round());
        }
        for (p, n) in &samples {
            let off = p.sub(centre);
            let radial = off.sub(axis.scale(off.dot(axis)));
            if n.dot(axis).abs() > 1e-9 || (radial.length() - radius).abs() > tolerance
                || radial.scale(outward / radius).sub(*n).length() > 1e-9 {
                return Err(not_round());
            }
        }
        Ok((Shape::Round { centre, axis, radius, convex: outward > 0.0 }, boundary, lines))
    })?;
    Ok(Face { name: name.to_string(), solid, handle: picked.handle, face_id: picked.face_id, shape, boundary, lines })
}

/// Whether `point` lies on the picked face, inside its trim or on it.
pub(super) fn on_face(face: &Face, point: Vec3, tolerance: f64) -> Result<bool, String> {
    crate::with_registered_solid_str(face.handle, |body| {
        let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face.face_id)
            .ok_or_else(|| format!("face '{}' is no longer resident", face.name))?;
        Ok(matches!(fit::face_class(record, point, tolerance)?, Some(crate::PolygonClass::Inside | crate::PolygonClass::Boundary)))
    })
}

/// Whether `point` lies inside the body behind `handle`.
fn inside(handle: u32, point: Vec3) -> Result<bool, String> {
    crate::with_registered_solid_str(handle, |body| {
        Ok(crate::classify_point(point, body, slab_depth() * 1e-3)?.class == crate::PointClass::In)
    })
}

/// The groove's settings (read by the groove weld).
pub(super) struct Settings {
    pub flare: Flare,
    pub double: bool,
    pub cap: f64,
    pub generate: bool,
    pub colour: String,
}

pub(super) fn build(ctx: &FeatureContext, settings: Settings) -> Result<FeatureResult, String> {
    let Settings { flare, double, cap, generate, colour } = settings;
    let label = if flare == Flare::V { "flare-V" } else { "flare-bevel" };
    let names = common::unique_reference_names(ctx.param("seam"));
    let mut picked: Vec<String> = Vec::new();
    let mut edge: Option<String> = None;
    for name in &names {
        if !ctx.scene.resolve_face_group(name).is_empty() {
            picked.push(name.clone());
        } else if ctx.scene.resolve_edge(name).is_some() {
            if edge.replace(name.clone()).is_some() {
                return Err("select at most one edge (the side the groove opens to)".into());
            }
        } else {
            return Err(format!("reference '{name}' is not a resident face or edge"));
        }
    }
    let [first, second] = picked.as_slice() else {
        return Err(format!("select the two faces of the {label} groove"));
    };
    let mut faces = [face(ctx, first)?, face(ctx, second)?];
    for face in &faces {
        if let Shape::Round { convex: false, .. } = face.shape {
            return Err(format!("face '{}' is a concave round face (a bore); a {label} groove bears on the outside of a round bar or bend", face.name));
        }
    }
    if faces[0].handle == faces[1].handle {
        return Err(format!("the two faces belong to one part; a {label} groove joins two parts"));
    }
    // The flat face (a flare-bevel's) is the section's left side.
    match (flare, &faces[0].shape, &faces[1].shape) {
        (Flare::V, Shape::Round { .. }, Shape::Round { .. }) => {}
        (Flare::V, _, _) => return Err("a flare-V groove joins two round faces (bars or bends); for a round face against a flat one use FLARE_BEVEL".into()),
        (Flare::Bevel, Shape::Flat { .. }, Shape::Round { .. }) => {}
        (Flare::Bevel, Shape::Round { .. }, Shape::Flat { .. }) => faces.swap(0, 1),
        (Flare::Bevel, _, _) => return Err("a flare-bevel groove joins a round face (a bar or a bend) to a flat face; for two round faces use FLARE_V".into()),
    }
    let Shape::Round { centre: c2, axis: d, radius: r2, .. } = faces[1].shape.clone() else { unreachable!("checked above") };
    let size = faces.iter().flat_map(|f| &f.boundary).map(|p| p.sub(c2).length()).fold(1.0, f64::max);
    let tolerance = 1e-9 * size;
    // Across the joint: from the left face's root towards the round's axis.
    let (u, gap, left_round, origin_at_axis) = match faces[0].shape.clone() {
        Shape::Flat { on, normal } => {
            if normal.dot(d).abs() > 1e-9 {
                return Err(format!("the round face '{}' does not run parallel to the flat face '{}'", faces[1].name, faces[0].name));
            }
            let gap = c2.sub(on).dot(normal) - r2;
            (normal, gap, None, c2.sub(normal.scale(gap + r2)))
        }
        Shape::Round { centre: c1, axis, radius: r1, .. } => {
            if axis.cross(d).length() > 1e-9 {
                return Err(format!("the round faces '{}' and '{}' do not run parallel", faces[0].name, faces[1].name));
            }
            let between = c2.sub(c1);
            let across = between.sub(d.scale(between.dot(d)));
            let distance = across.length();
            if distance <= tolerance {
                return Err(format!("the round faces '{}' and '{}' share an axis", faces[0].name, faces[1].name));
            }
            let u = across.scale(1.0 / distance);
            (u, distance - r1 - r2, Some(r1), c1.add(u.scale(r1)))
        }
    };
    if gap < -tolerance {
        return Err(format!("faces '{}' and '{}' overlap by {}", faces[0].name, faces[1].name, -gap));
    }
    let gap = gap.max(0.0);
    let height = left_round.map_or(r2, |r1| r1.min(r2));
    // Along the seam: where both faces run.
    let extent = |points: &[Vec3]| points.iter().map(|p| p.sub(c2).dot(d))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| (lo.min(s), hi.max(s)));
    let (a0, a1) = extent(&faces[0].boundary);
    let (b0, b1) = extent(&faces[1].boundary);
    let (d0, d1) = (a0.max(b0), a1.min(b1));
    if d1 - d0 <= tolerance {
        return Err(format!("faces '{}' and '{}' do not overlap along the seam", faces[0].name, faces[1].name));
    }
    let length = d1 - d0;
    let origin = origin_at_axis.add(d.scale(d0 - origin_at_axis.sub(c2).dot(d)));
    let upward = u.cross(d).normalized()?;
    let depth = match common::optional_number(ctx, "flareDepth")? {
        None => 0.0,
        Some(value) if value.is_finite() && value >= 0.0 => value,
        Some(_) => return Err("groove depth must be finite and not negative".into()),
    };
    // Depths are read against the flare's own size, not the faces' extent.
    let local = 1e-9 * height.max(1.0);
    if depth > height + local {
        return Err(format!("the groove depth {depth} is deeper than the {height} flare"));
    }
    if depth > 0.0 && depth <= local {
        return Err(format!("the groove depth {depth} is too shallow to fill; 0 fills the flare down to its root"));
    }
    // The fill's bottom level: the root, or the depth below the top.
    let bottom = if depth > 0.0 { (height - depth).max(0.0) } else { 0.0 };
    let bottom = if bottom <= local { 0.0 } else { bottom };
    // Which sides the groove can open to: each picked face must carry the
    // fill's side along it, from the fill's bottom to the flare's top (a
    // bar's face runs round both sides; a bend's, or a face split along
    // the bar, only round its own).
    let middle = d.scale(length * 0.5);
    let on = 1e-6_f64.max(size * 1e-9);
    // Read at stations along the seam and across the fill's contact.
    let stations = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0];
    let carries = |sign: f64| -> Result<bool, String> {
        let w = upward.scale(sign);
        let mut rounds = vec![(1, c2, r2, u.scale(-1.0))];
        match (left_round, &faces[0].shape) {
            (Some(r1), Shape::Round { centre, .. }) => rounds.push((0, *centre, r1, u)),
            _ => {
                for along in stations {
                    for level in [bottom, (bottom + height) * 0.5, height] {
                        if !on_face(&faces[0], origin.add(d.scale(length * along)).add(w.scale(level)), on)? {
                            return Ok(false);
                        }
                    }
                }
            }
        }
        for (index, centre, radius, root) in rounds {
            let on_axis = centre.add(d.scale(origin.sub(centre).dot(d)));
            let (low, top) = ((bottom / radius).min(1.0).asin(), (height / radius).min(1.0).asin());
            for along in stations {
                for angle in [low, (low + top) * 0.5, top] {
                    let direction = root.scale(angle.cos()).add(w.scale(angle.sin()));
                    if !on_face(&faces[index], on_axis.add(d.scale(length * along)).add(direction.scale(radius)), on)? {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    };
    let sides: Vec<f64> = [1.0, -1.0].into_iter().map(|sign| carries(sign).map(|ok| ok.then_some(sign)))
        .collect::<Result<Vec<_>, _>>()?.into_iter().flatten().collect();
    let up = [upward.z, upward.y, upward.x].into_iter().find(|c| c.abs() > 1e-9).unwrap_or(1.0).signum();
    let chosen: Vec<f64> = if double {
        if sides.len() < 2 {
            return Err(format!("a double {label} groove needs the faces to run round both sides of the joint; a bend (or a face split along the bar) carries a groove on its own side only"));
        }
        vec![1.0, -1.0]
    } else if let Some(edge_name) = &edge {
        let curve = common::edge_curve(ctx.scene.resolve_edge(edge_name).expect("resolved above"))?;
        let [t0, t1] = curve.domain()?;
        let level = curve.evaluate((t0 + t1) * 0.5)?.sub(origin).dot(upward);
        if level.abs() <= tolerance {
            return Err(format!("edge '{edge_name}' lies on the joint's root level; pick an edge on the side the groove opens to"));
        }
        let sign = level.signum();
        if !sides.contains(&sign) {
            return Err(format!("the faces do not run round to the side of edge '{edge_name}'"));
        }
        vec![sign]
    } else {
        match sides.as_slice() {
            [] => return Err(format!("faces '{}' and '{}' make no {label} groove: on neither side do the faces run from the fill's bottom to the flare's top", faces[0].name, faces[1].name)),
            [only] => vec![*only],
            _ => vec![up],
        }
    };
    let section = Section { left_round, right: r2, gap, height, bottom, tolerance };
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let seam = Seam::line(origin, d, length, Some(faces[0].solid.clone()));
    let mut beads: Vec<(String, crate::BrepSolid)> = Vec::new();
    if generate {
        let regions = section.regions(&chosen, cap);
        for (n, (region, sign)) in regions.iter().enumerate() {
            let w = upward.scale(*sign);
            let at = |x: f64, y: f64| origin.add(u.scale(x)).add(w.scale(y));
            let name = if n == 0 { ctx.id.clone() } else { format!("{}:Bead{}", ctx.id, n + 1) };
            let outline: Vec<Side> = region.outline.iter().map(|side| side.place(&at)).collect();
            let mut bead = fit::sweep_outline(&seam, &outline, &Trim::default())?;
            for (i, face) in bead.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
                face.name = Some(format!("{name}:BeadFace{}", i + 1));
            }
            if let Err(reason) = fit::clear(ctx.scene, &bead, &ctx.id, 1e-6_f64.max(size * 1e-12))? {
                return Err(reason);
            }
            for (side, contact) in &region.contacts {
                let handle = faces[*side].handle;
                let inside = |q: [f64; 2]| inside(handle, at(q[0], q[1]).add(middle));
                let unbacked = || format!("the bead is not backed by {} along the whole joint", faces[*side].name);
                let Some(outline) = slab(contact, &inside)? else { return Err(unbacked()) };
                let outline: Vec<Side> = outline.iter().map(|side| side.place(&at)).collect();
                let slab = fit::sweep_outline(&seam, &outline, &Trim::default())?;
                let body = crate::registered_solid_clone(handle)?;
                if let Err(reason) = fit::backed(&slab, &body)? {
                    return Err(format!("{reason}: {}", unbacked()));
                }
            }
            beads.push((name, bead));
        }
    }
    for (name, bead) in beads {
        result.added.push(common::register_added(bead, &name));
        super::super::paint_bead(&name, &colour);
    }
    for face in &faces {
        if !result.weld_joint_solids.contains(&face.solid) {
            result.weld_joint_solids.push(face.solid.clone());
        }
    }
    // The seam along the middle of the flare's opening on the first side.
    let w = upward.scale(chosen[0]);
    let top = origin.add(u.scale(section.top_middle())).add(w.scale(height));
    let seam_curve = crate::make_line(top, top.add(d.scale(length)))?;
    result.weld_source_seam = Some(faces[0].name.clone());
    result.paths.push((format!("{}:Seam", ctx.id), vec![seam_curve]));
    result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(top.add(middle))));
    Ok(result)
}

/// The flare's section: the left face at x = 0 (a plane, or a round of
/// radius `left_round` centred at (−r1, 0)), the right round of radius
/// `right` centred at (gap + right, 0); filled from `bottom` up to `height`.
struct Section {
    left_round: Option<f64>,
    right: f64,
    gap: f64,
    height: f64,
    bottom: f64,
    tolerance: f64,
}

impl Section {
    /// The left face's side from level `y0` up to `y1` (part index 0).
    fn left(&self, y0: f64, y1: f64) -> Flat {
        match self.left_round {
            None => Flat::Line([0.0, y0], [0.0, y1]),
            Some(r) => round_arc([-r, 0.0], r, 1.0, y0, y1),
        }
    }

    /// The right face's side from level `y0` up to `y1` (part index 1).
    fn right(&self, y0: f64, y1: f64) -> Flat {
        round_arc([self.gap + self.right, 0.0], self.right, -1.0, y0, y1)
    }

    fn x_at(flat: &Flat, low: bool) -> [f64; 2] {
        match *flat {
            Flat::Line(a, b) => if low { a } else { b },
            Flat::Arc { from, to, .. } => if low { from } else { to },
        }
    }

    /// The middle of the flare's top across the joint.
    fn top_middle(&self) -> f64 {
        let (l, r) = (Self::x_at(&self.left(0.0, self.height), false), Self::x_at(&self.right(0.0, self.height), false));
        (l[0] + r[0]) * 0.5
    }

    /// The bodies of the fill, each with the side (+1/−1 along the section's
    /// y) it opens to: one per side, or one through the root opening when a
    /// double flare is filled to the root across an opening.
    fn regions(&self, signs: &[f64], cap: f64) -> Vec<(Region, f64)> {
        let (bottom, height) = (self.bottom, self.height);
        let left = self.left(bottom, height);
        let right = self.right(bottom, height);
        let (l0, l1) = (Self::x_at(&left, true), Self::x_at(&left, false));
        let (r0, r1) = (Self::x_at(&right, true), Self::x_at(&right, false));
        let top = |from: [f64; 2], to: [f64; 2]| if cap > 0.0 { arc_flat(from, to, cap) } else { Flat::Line(from, to) };
        let open_root = (r0[0] - l0[0]).abs() > self.tolerance;
        if signs.len() == 2 && open_root && bottom <= self.tolerance {
            // One body through the opening: the lower half is the upper
            // mirrored across the root level.
            let down = |flat: &Flat| super::mirror_y(flat, 0.0);
            let outline = vec![
                down(&right).reversed(),
                right,
                top(r1, l1),
                left.reversed(),
                down(&left),
                down(&top(r1, l1)).reversed(),
            ];
            let contacts = vec![(0, left), (0, down(&left)), (1, right), (1, down(&right))];
            return vec![(Region { outline, contacts }, 1.0)];
        }
        signs.iter().map(|sign| {
            let mut outline = vec![right, top(r1, l1), left.reversed()];
            if open_root {
                outline.push(Flat::Line(l0, r0));
            }
            (Region { outline, contacts: vec![(0, left), (1, right)] }, *sign)
        }).collect()
    }
}

/// The arc of the round centred at `centre` (on the root level) of radius
/// `radius` facing `facing` (+1: its right side, −1: its left), from level
/// `y0` up to `y1`.
fn round_arc(centre: [f64; 2], radius: f64, facing: f64, y0: f64, y1: f64) -> Flat {
    let point = |angle: f64| [centre[0] + facing * radius * angle.cos(), centre[1] + radius * angle.sin()];
    let (a0, a1) = ((y0 / radius).clamp(-1.0, 1.0).asin(), (y1 / radius).clamp(-1.0, 1.0).asin());
    Flat::Arc { from: point(a0), to: point(a1), centre, through: point((a0 + a1) * 0.5) }
}
