//! Edge, outside-corner and surfacing welds: beads laid on the outside of
//! parts. An edge weld caps the flush edges of two parallel members; a flush
//! outside-corner weld rounds over the corner of two members; a surfacing
//! weld lays a layer on a flat face. (A half-open corner's notch is a concave
//! corner, filled by the fillet weld; a back or backing weld is the groove
//! weld's root reinforcement.)
use super::super::fit::{self, Side, Trim};
use super::{arc_flat, slab, slab_depth, Flat};
use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureResult, SceneMap, ScenePoint};
use crate::{BrepSolid, PointClass, Vec3};

fn number(ctx: &FeatureContext, key: &str, default: f64, label: &str, positive: bool) -> Result<f64, String> {
    let value = common::optional_number(ctx, key)?.unwrap_or(default);
    if !value.is_finite() || value < 0.0 || (positive && value == 0.0) {
        return Err(format!("{label} must be finite and {}", if positive { "positive" } else { "not negative" }));
    }
    Ok(value)
}

fn generate(ctx: &FeatureContext) -> Result<bool, String> {
    match ctx.param("generateSolid") {
        None => Ok(true),
        Some(serde_json::Value::Bool(value)) => Ok(*value),
        Some(_) => Err("Generate weld solid must be a boolean".into()),
    }
}

fn colour(ctx: &FeatureContext) -> Result<String, String> {
    match ctx.param("weldColor") {
        None => Ok(super::super::WELD_COLOUR.to_string()),
        Some(serde_json::Value::String(hex)) if super::super::is_hex_colour(hex) => Ok(hex.to_ascii_lowercase()),
        Some(_) => Err("weld colour must be `#rrggbb`".into()),
    }
}

fn common_fields(fields: &mut serde_json::Value) {
    let more = serde_json::json!({
        "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
            "hint": "On by default: build the bead as a separate solid. Off keeps the weld and its callout."},
        "weldColor": {"type": "color", "label": "Weld colour", "default_value": "#d9822b",
            "hint": "Colour of the bead solid in the 3D view and exports."},
        "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
    });
    for (key, value) in more.as_object().expect("fields") {
        fields[key] = value.clone();
    }
}

fn body_of(scene: &SceneMap, handle: u32) -> Option<String> {
    scene.solids.iter().filter(|(_, other)| **other == handle).map(|(name, _)| name.clone()).min()
}

/// What of `body` lies outside every one of `parts`.
fn outside(body: &BrepSolid, parts: &[BrepSolid]) -> Result<f64, String> {
    let options = crate::BooleanOptions { merge_coplanar_faces: false, ..Default::default() };
    let mut rest = body.clone();
    for part in parts {
        match crate::boolean_operation(&rest, part, crate::BooleanOperation::Subtract, &options) {
            Ok(left) => rest = left,
            Err(refusal) if matches!(refusal.class, crate::RefusalClass::ConservativeEmptyOverlap) => return Ok(0.0),
            Err(refusal) => return Err(format!("could not check the bead against the parts: {refusal}")),
        }
    }
    crate::solid_mass_properties(&rest).map(|m| m.volume.abs()).map_err(|error| format!("could not measure the bead: {error}"))
}

fn finish(ctx: &FeatureContext, result: &mut FeatureResult, bodies: Vec<BrepSolid>, colour: &str) {
    for (n, mut body) in bodies.into_iter().enumerate() {
        let name = if n == 0 { ctx.id.clone() } else { format!("{}:Bead{}", ctx.id, n + 1) };
        for (i, face) in body.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
            face.name = Some(format!("{name}:BeadFace{}", i + 1));
        }
        result.added.push(common::register_added(body, &name));
        super::super::paint_bead(&name, colour);
    }
}

/// A flat face: its body, a point on its plane, its outward normal and the
/// points of its boundary.
struct Flat3 {
    solid: String,
    handle: u32,
    on: Vec3,
    normal: Vec3,
    corners: Vec<Vec3>,
    lines: Vec<(Vec3, Vec3)>,
}

fn flat_face(ctx: &FeatureContext, name: &str) -> Result<Flat3, String> {
    let group = ctx.scene.resolve_face_group(name);
    let [face] = group.as_slice() else { return Err(format!("face '{name}' stands for several faces; select one")) };
    let solid = body_of(ctx.scene, face.handle).ok_or_else(|| format!("face '{name}' belongs to no resident part"))?;
    crate::with_registered_solid_str(face.handle, |body| {
        let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face.face_id)
            .ok_or_else(|| format!("face '{name}' is no longer resident"))?;
        let mut corners = Vec::new();
        let mut lines = Vec::new();
        for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
            let Some(e) = body.edges.iter().find(|e| e.id == coedge.edge_id) else { continue };
            let (a, b) = (e.curve.evaluate(e.t0)?, e.curve.evaluate(e.t1)?);
            corners.push(a);
            corners.push(b);
            if e.curve.degree == 1 { lines.push((a, b)); }
        }
        let first = *corners.first().ok_or_else(|| format!("face '{name}' has no boundary"))?;
        let (on, normal) = crate::fillet::face_plane_normal(record, first).map_err(|refusal| refusal.to_string())?;
        let size = corners.iter().map(|p| p.sub(first).length()).fold(0.0, f64::max).max(1.0);
        let level = record.surface.control_points.iter().flatten().all(|pole| {
            let w = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
            Vec3::new(pole.x / w, pole.y / w, pole.z / w).sub(on).dot(normal).abs() <= 1e-9 * size
        });
        if !level {
            return Err(format!("face '{name}' is not flat"));
        }
        Ok(Flat3 { solid: solid.clone(), handle: face.handle, on, normal, corners, lines })
    })
}

fn extent(points: &[Vec3], origin: Vec3, axis: Vec3) -> (f64, f64) {
    points.iter().map(|p| p.sub(origin).dot(axis)).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| (lo.min(s), hi.max(s)))
}

/// Sweep a section outline (in its own x, y) placed by `at` along a straight
/// run `length` long in direction `d` from `origin`.
fn sweep(outline: &[Flat], at: &impl Fn(f64, f64) -> Vec3, origin: Vec3, d: Vec3, length: f64) -> Result<BrepSolid, String> {
    let seam = fit::Seam::line(origin, d, length, None);
    let sides: Vec<Side> = outline.iter().map(|flat| flat.place(at)).collect();
    fit::sweep_outline(&seam, &sides, &Trim::default())
}

/// Every contact side backed by the parts: a thin slab under it lies in
/// them (read halfway along the run).
fn backed(contacts: &[Flat], parts: &[BrepSolid], at: &impl Fn(f64, f64) -> Vec3, origin: Vec3, d: Vec3, length: f64, what: &str) -> Result<(), String> {
    for contact in contacts {
        let inside = |q: [f64; 2]| -> Result<bool, String> {
            let point = at(q[0], q[1]).add(d.scale(length * 0.5));
            for part in parts {
                if crate::classify_point(point, part, slab_depth() * 1e-3)?.class == PointClass::In { return Ok(true); }
            }
            Ok(false)
        };
        let Some(outline) = slab(contact, &inside)? else { return Err(format!("the bead is not backed by {what} along the whole weld")) };
        let slab = sweep(&outline, at, origin, d, length)?;
        let left = outside(&slab, parts)?;
        if left > 1e-9 {
            return Err(format!("{left} of the support slab lies outside {what}: the bead is not backed along the whole weld"));
        }
    }
    Ok(())
}

// --------------------------------------------------------------- edge weld

pub fn edge_schema() -> serde_json::Value {
    let mut schema = serde_json::json!({
        "type": "WELD.EDGE", "shortName": "EDGEW", "longName": "Edge weld",
        "ribbonPath": "Welding/Edge and surface/Edge weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["FACE"], "multiple": true, "default_value": null,
                "hint": "Select the two flush edge faces of the members lying side by side (flat, facing the same way, in one plane)."},
            "reinforcement": {"type": "number", "label": "Reinforcement", "default_value": 1,
                "hint": "Height of the bead over the two edge faces, in model units. The bead spans both faces."}
        }
    });
    common_fields(&mut schema["inputParamsSchema"]);
    schema
}

pub fn execute_edge(ctx: &FeatureContext) -> FeatureResult {
    edge(ctx).unwrap_or_else(|error| ctx.fail(format!("edge weld: {error}")))
}

fn edge(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let names = common::unique_reference_names(ctx.param("seam"));
    let [first, second] = names.as_slice() else { return Err("select the two flush edge faces".into()) };
    let height = number(ctx, "reinforcement", 1.0, "reinforcement", true)?;
    let colour = colour(ctx)?;
    let (a, b) = (flat_face(ctx, first)?, flat_face(ctx, second)?);
    if a.handle == b.handle {
        return Err("the two faces belong to one part; an edge weld joins two members".into());
    }
    let size = a.corners.iter().chain(&b.corners).map(|p| p.sub(a.on).length()).fold(1.0, f64::max);
    let tolerance = 1e-9 * size;
    if a.normal.dot(b.normal) < 1.0 - 1e-9 || b.on.sub(a.on).dot(a.normal).abs() > tolerance {
        return Err(format!("faces '{first}' and '{second}' are not flush (in one plane, facing the same way)"));
    }
    let n = a.normal;
    // Along the edges: the first face's longer straight side.
    let d = a.lines.iter().filter_map(|(p, q)| q.sub(*p).normalized().ok())
        .max_by(|x, y| {
            let (lx, hx) = extent(&a.corners, a.on, *x);
            let (ly, hy) = extent(&a.corners, a.on, *y);
            (hx - lx).total_cmp(&(hy - ly))
        }).ok_or_else(|| format!("face '{first}' has no straight side"))?;
    let mut u = n.cross(d).normalized()?;
    let centre = |points: &[Vec3]| points.iter().fold(Vec3::ZERO, |s, p| s.add(*p)).scale(1.0 / points.len() as f64);
    if centre(&b.corners).sub(centre(&a.corners)).dot(u) < 0.0 { u = u.scale(-1.0); }
    let (a0, a1) = extent(&a.corners, a.on, u);
    let (b0, b1) = extent(&b.corners, a.on, u);
    if a1 > b0 + tolerance {
        return Err(format!("faces '{first}' and '{second}' overlap across the edge"));
    }
    let (ad0, ad1) = extent(&a.corners, a.on, d);
    let (bd0, bd1) = extent(&b.corners, a.on, d);
    let (d0, d1) = (ad0.max(bd0), ad1.min(bd1));
    if d1 - d0 <= tolerance {
        return Err(format!("faces '{first}' and '{second}' do not run side by side"));
    }
    let origin = a.on.add(d.scale(d0));
    let at = |x: f64, y: f64| origin.add(u.scale(x)).add(n.scale(y));
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    result.weld_joint_solids = vec![a.solid.clone(), b.solid.clone()];
    result.weld_source_seam = Some(first.clone());
    let top = at((a0 + b1) * 0.5, height);
    result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(top.add(d.scale((d1 - d0) * 0.5)))));
    result.paths.push((format!("{}:Seam", ctx.id), vec![crate::make_line(at((a1 + b0) * 0.5, 0.0), at((a1 + b0) * 0.5, 0.0).add(d.scale(d1 - d0)))?]));
    if generate(ctx)? {
        // The cap: across both faces, from the first's far side to the second's.
        let outline = vec![Flat::Line([b1, 0.0], [a0, 0.0]), arc_flat([a0, 0.0], [b1, 0.0], height)];
        let bead = sweep(&outline, &at, origin, d, d1 - d0)?;
        if let Err(reason) = fit::clear_of(ctx.scene, &bead, &ctx.id, &[], tolerance)? {
            return Err(reason);
        }
        let parts = [crate::registered_solid_clone(a.handle)?, crate::registered_solid_clone(b.handle)?];
        backed(&[Flat::Line([a0, 0.0], [a1, 0.0])], &parts[..1], &at, origin, d, d1 - d0, &a.solid)?;
        backed(&[Flat::Line([b0, 0.0], [b1, 0.0])], &parts[1..], &at, origin, d, d1 - d0, &b.solid)?;
        finish(ctx, &mut result, vec![bead], &colour);
    }
    Ok(result)
}

// ------------------------------------------------------ outside corner weld

pub fn corner_schema() -> serde_json::Value {
    let mut schema = serde_json::json!({
        "type": "WELD.CORNER", "shortName": "CORNERW", "longName": "Corner weld",
        "ribbonPath": "Welding/Edge and surface/Corner weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE"], "multiple": true, "default_value": null,
                "hint": "FLUSH: select the outside corner's edge. HALF_OPEN: select the edge at the bottom of the notch the half-open corner leaves."},
            "corner": {"type": "options", "label": "Corner", "options": ["FLUSH", "HALF_OPEN"], "default_value": "FLUSH",
                "hint": "FLUSH rounds a bead over the outside corner. HALF_OPEN fills the notch a half-open corner leaves, as a fillet weld fills a concave corner."},
            "size": {"type": "number", "label": "Size", "default_value": 3,
                "hint": "FLUSH: the radius the bead rounds over the corner with, along each face. HALF_OPEN: the fillet's leg."}
        }
    });
    common_fields(&mut schema["inputParamsSchema"]);
    schema
}

pub fn execute_corner(ctx: &FeatureContext) -> FeatureResult {
    corner(ctx).unwrap_or_else(|error| ctx.fail(format!("corner weld: {error}")))
}

fn corner(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let size = number(ctx, "size", 3.0, "size", true)?;
    match ctx.param("corner").and_then(serde_json::Value::as_str).unwrap_or("FLUSH") {
        "FLUSH" => {}
        "HALF_OPEN" => {
            // The notch is a concave corner: the fillet weld fills it, fitted
            // and checked as fillet beads are.
            let mut params = ctx.params.clone();
            params["legSize"] = size.into();
            params["placement"] = "FITTED".into();
            let fillet = FeatureContext { id: ctx.id.clone(), feature_type: ctx.feature_type.clone(), params: &params,
                persistent: ctx.persistent, env: ctx.env, scene: ctx.scene, later_inputs: ctx.later_inputs, rejoin_references: ctx.rejoin_references };
            let mut result = super::super::build(&fillet, None)?;
            result.feature_type = ctx.feature_type.clone();
            return Ok(result);
        }
        other => return Err(format!("unknown corner '{other}' (FLUSH or HALF_OPEN)")),
    }
    let names = common::unique_reference_names(ctx.param("seam"));
    let [name] = names.as_slice() else { return Err("select the outside corner's edge".into()) };
    let reference = ctx.scene.resolve_edge(name).ok_or_else(|| format!("'{name}' is not a resident edge"))?;
    let curve = common::edge_curve(reference)?;
    let [t0, t1] = curve.domain()?;
    let (start, end) = (curve.evaluate(t0)?, curve.evaluate(t1)?);
    if curve.degree != 1 && curve.control_points.len() != 2 {
        let middle = curve.evaluate((t0 + t1) * 0.5)?;
        if middle.sub(start.add(end).scale(0.5)).length() > 1e-9 * (1.0 + start.length()) {
            return Err("a flush corner weld follows a straight corner edge".into());
        }
    }
    let d = end.sub(start).normalized()?;
    let length = end.sub(start).length();
    // The two faces of the corner: the faces of the edge's body that meet
    // along it, with their outward normals there.
    let normals: Vec<Vec3> = crate::with_registered_solid_str(reference.handle, |body| {
        let mut normals = Vec::new();
        for face in body.shells.iter().flat_map(|shell| &shell.faces) {
            if !face.loops.iter().flat_map(|l| &l.coedges).any(|c| c.edge_id == reference.edge_id) { continue; }
            let middle = start.add(end).scale(0.5);
            let (_, outward) = crate::fillet::face_plane_normal(face, middle).map_err(|refusal| refusal.to_string())?;
            normals.push(outward);
        }
        Ok(normals)
    })?;
    let [n1, n2] = normals.as_slice() else { return Err(format!("edge '{name}' does not join two faces")) };
    // Along each face, away from the edge; convex: each leg points against
    // the other face's normal.
    let leg = |n: Vec3, other: Vec3| -> Result<Vec3, String> {
        let l = n.cross(d).normalized()?;
        Ok(if l.dot(other) > 0.0 { l.scale(-1.0) } else { l })
    };
    let (l1, l2) = (leg(*n1, *n2)?, leg(*n2, *n1)?);
    if l1.dot(*n2) > -1e-9 || l2.dot(*n1) > -1e-9 {
        return Err(format!("edge '{name}' is not an outside (convex) corner"));
    }
    let out = n1.add(*n2).normalized()?;
    // The section in the plane square to the edge: x along the first leg, y
    // along the second leg's component square to it.
    let x_axis = l1;
    let y_axis = l2.sub(l1.scale(l2.dot(l1))).normalized()?;
    let at = |x: f64, y: f64| start.add(x_axis.scale(x)).add(y_axis.scale(y));
    let to2 = |v: Vec3| [v.dot(x_axis), v.dot(y_axis)];
    let (p1, p2, apex) = (to2(l1.scale(size)), to2(l2.scale(size)), to2(out.scale(size)));
    let outline = vec![
        Flat::Line(p1, [0.0, 0.0]),
        Flat::Line([0.0, 0.0], p2),
        Flat::Arc { from: p2, to: p1, centre: [0.0, 0.0], through: apex },
    ];
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let owner = body_of(ctx.scene, reference.handle).ok_or("the edge belongs to no resident part")?;
    result.weld_source_seam = Some(name.clone());
    result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(start.add(end).scale(0.5).add(out.scale(size)))));
    result.paths.push((format!("{}:Seam", ctx.id), vec![crate::make_line(start, end)?]));
    // The parts the bead lies on: every body a leg's strip touches.
    let mut parts: Vec<(String, BrepSolid)> = Vec::new();
    for (name, handle) in &ctx.scene.solids {
        if fit::owned_by(name, &ctx.id) { continue; }
        let body = crate::registered_solid_clone(*handle)?;
        let touches = [l1, l2].iter().any(|l| {
            let probe = start.add(end).scale(0.5).add(l.scale(size * 0.5)).sub(out.scale(1e-4));
            crate::classify_point(probe, &body, 1e-7).is_ok_and(|c| c.class != PointClass::Out)
        });
        if touches || *name == owner { parts.push((name.clone(), body)); }
    }
    result.weld_joint_solids = parts.iter().map(|(name, _)| name.clone()).collect();
    if generate(ctx)? {
        let colour = colour(ctx)?;
        let bead = sweep(&outline, &at, start, d, length)?;
        let tolerance = 1e-9 * (1.0 + length);
        if let Err(reason) = fit::clear_of(ctx.scene, &bead, &ctx.id, &[], tolerance)? {
            return Err(reason);
        }
        let bodies: Vec<BrepSolid> = parts.into_iter().map(|(_, body)| body).collect();
        backed(&[Flat::Line(p1, [0.0, 0.0]), Flat::Line([0.0, 0.0], p2)], &bodies, &at, start, d, length, "the corner's members")?;
        finish(ctx, &mut result, vec![bead], &colour);
    }
    Ok(result)
}

// -------------------------------------------------------- surfacing weld

/// A surfacing layer on a cylindrical face: the face's band or patch of its
/// cylinder thickened `thickness` away from the part (outward on a shaft or
/// a bend's outside, inward in a bore), revolved exactly about the axis over
/// the face's angle; with the support slab the same depth inside the part
/// and the callout's point. `None` when the face is not cylindrical.
fn round_layer(ctx: &FeatureContext, name: &str, thickness: f64) -> Result<Option<(BrepSolid, BrepSolid, Vec3)>, String> {
    use super::flare::{face, on_face, Shape};
    let Ok(picked) = face(ctx, name) else { return Ok(None) };
    let Shape::Round { centre, axis, radius, convex } = picked.shape.clone() else { return Ok(None) };
    let (inner, outer) = if convex { (radius, radius + thickness) } else { (radius - thickness, radius) };
    let size = picked.boundary.iter().map(|p| p.sub(centre).length()).fold(radius.max(1.0), f64::max);
    let tolerance = 1e-9 * size;
    if inner <= tolerance {
        return Err(format!("a {thickness} thick layer fills the {radius} radius bore of '{name}'"));
    }
    // The face's run along the axis and round it, from its boundary.
    let first = picked.boundary.iter().map(|p| { let off = p.sub(centre); off.sub(axis.scale(off.dot(axis))) })
        .find(|radial| radial.length() > tolerance).ok_or_else(|| format!("face '{name}' has no boundary off its axis"))?;
    let x = first.normalized()?;
    let y = axis.cross(x);
    let (mut z0, mut z1) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &picked.boundary {
        let z = p.sub(centre).dot(axis);
        (z0, z1) = (z0.min(z), z1.max(z));
    }
    let tau = std::f64::consts::TAU;
    let angle_of = |p: Vec3| -> Option<f64> {
        let off = p.sub(centre);
        let radial = off.sub(axis.scale(off.dot(axis)));
        (radial.length() > tolerance).then(|| radial.dot(y).atan2(radial.dot(x)).rem_euclid(tau))
    };
    let at = |angle: f64, z: f64, r: f64| centre.add(axis.scale(z)).add(x.scale(angle.cos() * r)).add(y.scale(angle.sin() * r));
    let on = 1e-6_f64.max(size * 1e-9);
    let middle = (z0 + z1) * 0.5;
    // A patch ends at its straight edges along the axis (a full band has
    // none, or only its seam line): their angles.
    let mut ends: Vec<f64> = Vec::new();
    for (a, b) in &picked.lines {
        let Ok(along) = b.sub(*a).normalized() else { continue };
        if along.cross(axis).length() > 1e-9 { continue; }
        let Some(angle) = angle_of(*a) else { continue };
        let same = |known: &f64| { let d = (known - angle).rem_euclid(tau); d.min(tau - d) <= 1e-9 };
        if !ends.iter().any(same) { ends.push(angle); }
    }
    let (start, sweep) = match ends.as_slice() {
        [] | [_] => (ends.first().copied().unwrap_or(0.0), tau),
        [a, b] => {
            // The patch is the arc between them its middle lies on.
            let forward = (b - a).rem_euclid(tau);
            if on_face(&picked, at(a + forward * 0.5, middle, radius), on)? {
                (*a, forward)
            } else {
                (*b, tau - forward)
            }
        }
        _ => return Err(format!("face '{name}' is not a whole band or a rectangular patch of its cylinder (it has more than two straight edges along its axis); a curved surfacing weld covers those")),
    };
    // The face fills that band or patch (no hole, no slanted end).
    for i in 0..=4 {
        for j in 0..=4 {
            let (angle, z) = (start + sweep * (0.05 + 0.9 * i as f64 / 4.0), z0 + (z1 - z0) * (0.05 + 0.9 * j as f64 / 4.0));
            if !on_face(&picked, at(angle, z, radius), on)? {
                return Err(format!("face '{name}' is not a whole band or a rectangular patch of its cylinder (a hole, or ends not square to the axis); a curved surfacing weld covers those"));
            }
        }
    }
    let shell = |from: f64, to: f64| -> Result<BrepSolid, String> {
        let corners = [at(start, z0, from), at(start, z0, to), at(start, z1, to), at(start, z1, from)];
        let profile = (0..4).map(|k| crate::make_line(corners[k], corners[(k + 1) % 4])).collect::<Result<Vec<_>, _>>()?;
        crate::revolve_profile_brep(&profile, centre, axis, sweep)
    };
    let layer = shell(inner, outer)?;
    let slab = if convex { shell(radius - slab_depth(), radius)? } else { shell(radius, radius + slab_depth())? };
    let callout = at(start + sweep * 0.5, middle, if convex { outer } else { inner });
    Ok(Some((layer, slab, callout)))
}

pub fn surfacing_schema() -> serde_json::Value {
    let mut schema = serde_json::json!({
        "type": "WELD.SURF", "shortName": "SURF", "longName": "Surfacing weld",
        "ribbonPath": "Welding/Edge and surface/Surfacing weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["FACE"], "multiple": true, "default_value": null,
                "hint": "Select the faces to build up with weld metal (hardfacing, build-up): flat faces (holes stay open), or cylindrical ones (a shaft's or a bend's outside, a bore's inside: a whole band or a rectangular patch)."},
            "thickness": {"type": "number", "label": "Thickness", "default_value": 2, "hint": "Thickness of the surfacing layer, in model units."}
        }
    });
    common_fields(&mut schema["inputParamsSchema"]);
    schema
}

pub fn execute_surfacing(ctx: &FeatureContext) -> FeatureResult {
    surfacing(ctx).unwrap_or_else(|error| ctx.fail(format!("surfacing weld: {error}")))
}

fn surfacing(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let thickness = number(ctx, "thickness", 2.0, "thickness", true)?;
    let names = common::unique_reference_names(ctx.param("seam"));
    if names.is_empty() {
        return Err("select the faces to surface".into());
    }
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let mut bodies = Vec::new();
    let build = generate(ctx)?;
    for (n, name) in names.iter().enumerate() {
        let group = ctx.scene.resolve_face_group(name);
        let [face] = group.as_slice() else { return Err(format!("face '{name}' stands for several faces; select one")) };
        let solid = body_of(ctx.scene, face.handle).ok_or_else(|| format!("face '{name}' belongs to no resident part"))?;
        if !result.weld_joint_solids.contains(&solid) { result.weld_joint_solids.push(solid.clone()); }
        if let Some((layer, slab, callout)) = round_layer(ctx, name, thickness)? {
            if n == 0 {
                result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(callout)));
                result.weld_source_seam = Some(name.clone());
            }
            if !build { continue; }
            if let Err(reason) = fit::clear_of(ctx.scene, &layer, &ctx.id, &[], 1e-6)? {
                return Err(format!("face '{name}': {reason}"));
            }
            for (k, earlier) in bodies.iter().enumerate() {
                let shared = fit::common_volume(&layer, earlier)?;
                if shared > 1e-9 {
                    return Err(format!("the layers on '{name}' and '{}' overlap by {shared}; surface them as separate welds", names[k]));
                }
            }
            let left = outside(&slab, &[crate::registered_solid_clone(face.handle)?])?;
            if left > 1e-9 {
                return Err(format!("face '{name}': {left} of the support slab lies outside {solid}"));
            }
            bodies.push(layer);
            continue;
        }
        // A face that is neither (the round ones were taken above).
        let flat = flat_face(ctx, name).map_err(|error| if error.ends_with("is not flat") {
            format!("face '{name}' is neither flat nor cylindrical")
        } else {
            error
        })?;
        // The face's loops, each as its curves head to tail.
        let loops: Vec<Vec<crate::NurbsCurve>> = crate::with_registered_solid_str(face.handle, |body| {
            let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face.face_id)
                .ok_or_else(|| format!("face '{name}' is no longer resident"))?;
            let mut loops = Vec::new();
            for lp in &record.loops {
                let mut curves = Vec::new();
                for coedge in &lp.coedges {
                    let edge = body.edges.iter().find(|e| e.id == coedge.edge_id).ok_or("a loop edge is missing")?;
                    let curve = common::trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
                    curves.push(if coedge.forward { curve } else { curve.reversed()? });
                }
                loops.push(curves);
            }
            Ok(loops)
        })?;
        // The outer loop encloses every other, so its extent in the face's
        // plane is the largest (read from points densely on the curves,
        // never their poles, which can stand outside the boundary).
        let (e1, e2) = {
            let e1 = flat.normal.perpendicular()?;
            (e1, flat.normal.cross(e1))
        };
        let reach = |curves: &Vec<crate::NurbsCurve>| -> Result<f64, String> {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for curve in curves {
                let [t0, t1] = curve.domain()?;
                for k in 0..=512 {
                    let p = curve.evaluate(t0 + (t1 - t0) * k as f64 / 512.0)?.sub(flat.on);
                    for (i, axis) in [e1, e2].iter().enumerate() {
                        lo[i] = lo[i].min(p.dot(*axis));
                        hi[i] = hi[i].max(p.dot(*axis));
                    }
                }
            }
            Ok((hi[0] - lo[0]) + (hi[1] - lo[1]))
        };
        let reaches = loops.iter().map(&reach).collect::<Result<Vec<f64>, String>>()?;
        let outer = (0..loops.len()).max_by(|a, b| reaches[*a].total_cmp(&reaches[*b])).ok_or("the face has no loop")?;
        let centre = flat.corners.iter().fold(Vec3::ZERO, |s, p| s.add(*p)).scale(1.0 / flat.corners.len().max(1) as f64);
        if n == 0 {
            result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(centre.add(flat.normal.scale(thickness)))));
            result.weld_source_seam = Some(name.clone());
        }
        if !build { continue; }
        let layer_of = |depth: f64, direction: Vec3| -> Result<BrepSolid, String> {
            let mut layer = crate::extrude_profile_brep(&loops[outer], direction, depth).map_err(|error| error.to_string())?;
            for (k, inner) in loops.iter().enumerate() {
                if k == outer { continue; }
                // A hole's plug, a hair longer than the layer at both ends.
                let hair = depth * 0.01 + 100.0 * crate::tolerance::WELD_FLOOR;
                let back = direction.scale(-hair);
                let shift = crate::AffineTransform::new([1., 0., 0., back.x, 0., 1., 0., back.y, 0., 0., 1., back.z, 0., 0., 0., 1.])?;
                let moved: Vec<crate::NurbsCurve> = inner.iter().map(|c| crate::transform_topology::transform_curve(c, shift))
                    .collect::<Result<_, _>>()?;
                let plug = crate::extrude_profile_brep(&moved, direction, depth + 2.0 * hair).map_err(|error| error.to_string())?;
                layer = common::subtract_solid(layer, plug).map_err(|refusal| format!("could not open a hole in the layer: {refusal}"))?;
            }
            Ok(layer)
        };
        let layer = layer_of(thickness, flat.normal)?;
        if let Err(reason) = fit::clear_of(ctx.scene, &layer, &ctx.id, &[], 1e-6)? {
            return Err(format!("face '{name}': {reason}"));
        }
        // Nor with another picked face's layer (two faces of an inside corner).
        for (k, earlier) in bodies.iter().enumerate() {
            let shared = fit::common_volume(&layer, earlier)?;
            if shared > 1e-9 {
                return Err(format!("the layers on '{name}' and '{}' overlap by {shared}; surface them as separate welds", names[k]));
            }
        }
        let slab = layer_of(slab_depth(), flat.normal.scale(-1.0))?;
        let part = crate::registered_solid_clone(face.handle)?;
        let left = outside(&slab, &[part])?;
        if left > 1e-9 {
            return Err(format!("face '{name}': {left} of the support slab lies outside {solid}"));
        }
        bodies.push(layer);
    }
    finish(ctx, &mut result, bodies, &colour(ctx)?);
    Ok(result)
}
