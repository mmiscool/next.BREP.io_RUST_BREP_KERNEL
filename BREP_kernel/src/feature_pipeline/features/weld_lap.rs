//! Lap welds between two sheets or plates, one on the other: plug and slot
//! welds (a hole or slot in the top plate filled down to the lower one),
//! resistance spot welds (nuggets at a pitch along a line) and resistance
//! seam welds (one continuous nugget strip along a line). A nugget is metal
//! fused inside the lap, so it is checked by lying wholly inside the two
//! sheets rather than clear of them.
use super::common;
use super::fit::{self, Side};
use crate::feature_pipeline::{FeatureContext, FeatureResult, SceneMap, ScenePoint, SelectionProbe};
use crate::{BrepSolid, PointClass, PolygonClass, Vec2, Vec3};

fn number(ctx: &FeatureContext, key: &str, default: f64, label: &str, positive: bool) -> Result<f64, String> {
    let value = common::optional_number(ctx, key)?.unwrap_or(default);
    if !value.is_finite() || value < 0.0 || (positive && value == 0.0) {
        return Err(format!("{label} must be finite and {}", if positive { "positive" } else { "not negative" }));
    }
    Ok(value)
}

fn boolean(ctx: &FeatureContext, key: &str, default: bool, label: &str) -> Result<bool, String> {
    match ctx.param(key) {
        None => Ok(default),
        Some(serde_json::Value::Bool(value)) => Ok(*value),
        Some(_) => Err(format!("{label} must be a boolean")),
    }
}

fn colour(ctx: &FeatureContext) -> Result<String, String> {
    match ctx.param("weldColor") {
        None => Ok(super::WELD_COLOUR.to_string()),
        Some(serde_json::Value::String(hex)) if super::is_hex_colour(hex) => Ok(hex.to_ascii_lowercase()),
        Some(_) => Err("weld colour must be `#rrggbb`".into()),
    }
}

fn common_fields() -> serde_json::Value {
    serde_json::json!({
        "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
            "hint": "On by default: build the weld metal as a separate solid. Off keeps the weld and its callout."},
        "weldColor": {"type": "color", "label": "Weld colour", "default_value": "#d9822b",
            "hint": "Colour of the weld solid in the 3D view and exports."},
        "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
    })
}

fn with_common(mut fields: serde_json::Value) -> serde_json::Value {
    for (key, value) in common_fields().as_object().expect("fields") {
        fields[key] = value.clone();
    }
    fields
}

/// The resident body holding a face, by name.
fn body_of(scene: &SceneMap, handle: u32) -> Option<String> {
    scene.solids.iter().filter(|(_, other)| **other == handle).map(|(name, _)| name.clone()).min()
}

/// The volume left of `body` once every one of `parts` is removed: what of it
/// lies outside them all.
fn outside(body: &BrepSolid, parts: &[&BrepSolid]) -> Result<f64, String> {
    let options = crate::BooleanOptions { merge_coplanar_faces: false, ..Default::default() };
    let mut rest = body.clone();
    for part in parts {
        match crate::boolean_operation(&rest, part, crate::BooleanOperation::Subtract, &options) {
            Ok(left) => rest = left,
            // Nothing left: the rest lay wholly inside the part (the boolean
            // refuses an empty result as a conservative empty overlap).
            Err(refusal) if matches!(refusal.class, crate::RefusalClass::ConservativeEmptyOverlap) => return Ok(0.0),
            Err(refusal) => return Err(format!("could not check the weld metal against the parts: {refusal}")),
        }
    }
    crate::solid_mass_properties(&rest).map(|m| m.volume.abs()).map_err(|error| format!("could not measure the weld metal: {error}"))
}

/// The weld metal shares no volume with any resident body.
fn clear(scene: &SceneMap, id: &str, metal: &BrepSolid) -> Result<(), String> {
    match fit::clear_of(scene, metal, id, &[], 1e-6)? {
        Ok(()) => Ok(()),
        Err(reason) => Err(reason),
    }
}

fn register(result: &mut FeatureResult, id: &str, bodies: Vec<BrepSolid>, colour: &str) {
    for (n, mut body) in bodies.into_iter().enumerate() {
        let name = if n == 0 { id.to_string() } else { format!("{id}:Bead{}", n + 1) };
        for (i, face) in body.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
            face.name = Some(format!("{name}:BeadFace{}", i + 1));
        }
        result.added.push(common::register_added(body, &name));
        super::paint_bead(&name, colour);
    }
}

// ---------------------------------------------------------------- plug / slot

pub fn plug_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD.PLUG", "shortName": "PLUG", "longName": "Plug or slot weld",
        "ribbonPath": "Welding/Sheet metal and lap/Plug or slot weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": with_common(serde_json::json!({
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["FACE"], "multiple": true, "default_value": null,
                "hint": "Select the wall of each hole or slot in the top plate (any face of it). Each is filled from the plate under it."},
            "fillDepth": {"type": "number", "label": "Fill depth", "default_value": 0,
                "hint": "How deep the weld fills the hole from the plate under it, in model units. 0 = flush with the top plate's face."}
        }))
    })
}

pub fn execute_plug(ctx: &FeatureContext) -> FeatureResult {
    plug(ctx).unwrap_or_else(|error| ctx.fail(format!("plug weld: {error}")))
}

/// A hole's outline where the top plate meets the plate under it: the
/// closed loop's curves head to tail, its plane's outward normal (out of the
/// top plate, into the lower one), the top plate's thickness there and the
/// two bodies.
struct Hole {
    /// The bottom outline's part and edges: the same for every wall of one hole.
    key: (u32, Vec<u64>),
    outline: Vec<crate::NurbsCurve>,
    normal: Vec3,
    thickness: f64,
    plate: String,
    under: String,
    centre: Vec3,
}

fn hole(ctx: &FeatureContext, name: &str) -> Result<Hole, String> {
    let group = ctx.scene.resolve_face_group(name);
    let [wall] = group.as_slice() else { return Err(format!("face '{name}' stands for several faces; select one")) };
    let plate = body_of(ctx.scene, wall.handle).ok_or_else(|| format!("face '{name}' belongs to no resident part"))?;
    // Every planar face next to the wall bounds the hole with one of its
    // loops: the plate's two faces the hole runs between.
    let loops: Vec<(Vec3, Vec3, Vec<crate::NurbsCurve>, Vec3, Vec<u64>)> = crate::with_registered_solid_str(wall.handle, |body| {
        let faces: Vec<_> = body.shells.iter().flat_map(|shell| &shell.faces).collect();
        let record = faces.iter().find(|face| face.id == wall.face_id).ok_or_else(|| format!("face '{name}' is no longer resident"))?;
        let wall_edges: Vec<u64> = record.loops.iter().flat_map(|l| &l.coedges).map(|c| c.edge_id).collect();
        let mut found = Vec::new();
        for face in &faces {
            // A plate face the hole runs through has the hole as one of
            // several loops; a neighbouring wall has its outline alone.
            if face.id == wall.face_id || face.loops.len() < 2 { continue; }
            for lp in &face.loops {
                if !lp.coedges.iter().any(|c| wall_edges.contains(&c.edge_id)) { continue; }
                let mut curves = Vec::new();
                let mut points = Vec::new();
                for coedge in &lp.coedges {
                    let edge = body.edges.iter().find(|e| e.id == coedge.edge_id).ok_or("a loop edge is missing")?;
                    let curve = common::trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
                    let curve = if coedge.forward { curve } else { curve.reversed()? };
                    let [t0, t1] = curve.domain()?;
                    points.push(curve.evaluate((t0 + t1) * 0.5)?);
                    points.push(curve.evaluate(t0)?);
                    curves.push(curve);
                }
                let Some(&first) = points.first() else { continue };
                let Ok((on, outward)) = crate::fillet::face_plane_normal(face, first) else { continue };
                // The plate's faces stand square to the wall where they meet
                // it; a neighbouring wall face (a slot's side) does not.
                let shared = lp.coedges.iter().find(|c| wall_edges.contains(&c.edge_id)).expect("a shared edge");
                let edge = body.edges.iter().find(|e| e.id == shared.edge_id).ok_or("a loop edge is missing")?;
                let middle = edge.curve.evaluate((edge.t0 + edge.t1) * 0.5)?;
                let Ok((_, wall_normal)) = crate::fillet::face_plane_normal(record, middle) else { continue };
                if wall_normal.dot(outward).abs() > 1e-6 { continue; }
                let flat = points.iter().all(|p| p.sub(on).dot(outward).abs() <= 1e-7 * (1.0 + p.length()));
                if !flat { continue; }
                let centre = points.iter().fold(Vec3::ZERO, |sum, p| sum.add(*p)).scale(1.0 / points.len() as f64);
                let mut ids: Vec<u64> = lp.coedges.iter().map(|c| c.edge_id).collect();
                ids.sort_unstable();
                found.push((on, outward, curves, centre, ids));
            }
        }
        Ok(found)
    })?;
    if loops.len() != 2 {
        return Err(format!("face '{name}' is not the wall of a hole through a plate (it meets {} flat faces' outlines)", loops.len()));
    }
    // The bottom: the face whose far side, past the hole's middle, is metal.
    let mut bottom = None;
    for (index, (on, outward, _, centre, _)) in loops.iter().enumerate() {
        // Just past the face's plane, in the hole's middle.
        let probe = centre.sub(outward.scale(centre.sub(*on).dot(*outward))).add(outward.scale(1e-4));
        for (other, handle) in &ctx.scene.solids {
            if *other == plate || super::fit::owned_by(other, &ctx.id) { continue; }
            let inside = crate::with_registered_solid_str(*handle, |body| {
                Ok(crate::classify_point(probe, body, 1e-7)?.class == PointClass::In)
            })?;
            if inside {
                bottom = Some((index, other.clone()));
            }
        }
    }
    let Some((index, under)) = bottom else {
        return Err(format!("no plate lies under the hole of '{name}'; a plug weld fills a hole down to the plate under it"));
    };
    let (on, outward, curves, centre, ids) = loops[index].clone();
    let (top_on, _, _, _, _) = &loops[1 - index];
    let thickness = top_on.sub(on).dot(outward).abs();
    let level = centre.sub(on).dot(outward);
    Ok(Hole { key: (wall.handle, ids), outline: curves, normal: outward, thickness, plate, under, centre: centre.sub(outward.scale(level)) })
}

fn plug(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let names = common::unique_reference_names(ctx.param("seam"));
    if names.is_empty() {
        return Err("select the wall of the hole or slot to fill".into());
    }
    let depth = number(ctx, "fillDepth", 0.0, "fill depth", false)?;
    let generate = boolean(ctx, "generateSolid", true, "Generate weld solid")?;
    let colour = colour(ctx)?;
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let mut bodies = Vec::new();
    let mut joints: Vec<String> = Vec::new();
    let mut seen: Vec<(u32, Vec<u64>)> = Vec::new();
    let mut n = 0;
    for name in &names {
        let hole = hole(ctx, name)?;
        // Two walls of one hole (a slot's two ends) fill it once.
        if seen.contains(&hole.key) { continue; }
        seen.push(hole.key.clone());
        n += 1;
        let n = n - 1;
        let fill = if depth == 0.0 { hole.thickness } else { depth };
        if fill > hole.thickness * (1.0 + 1e-9) {
            return Err(format!("fill depth {fill} is deeper than the {} thick plate", hole.thickness));
        }
        result.points.push((format!("{}:Plug{}", ctx.id, n + 1), ScenePoint::model(hole.centre)));
        if n == 0 {
            result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(hole.centre.sub(hole.normal.scale(fill)))));
        }
        for body in [&hole.plate, &hole.under] {
            if !joints.contains(body) { joints.push(body.clone()); }
        }
        if !generate { continue; }
        let metal = crate::extrude_profile_brep(&hole.outline, hole.normal.scale(-1.0), fill).map_err(|error| error.to_string())?;
        clear(ctx.scene, &ctx.id, &metal).map_err(|reason| format!("hole '{name}': {reason}"))?;
        // The plate under it backs the whole bottom: a thin slab under it
        // lies wholly in that plate.
        let slab = crate::extrude_profile_brep(&hole.outline, hole.normal, 100.0 * crate::tolerance::WELD_FLOOR).map_err(|error| error.to_string())?;
        let under = crate::registered_solid_clone(ctx.scene.solids[&hole.under])?;
        let left = outside(&slab, &[&under])?;
        if left > 1e-9 {
            return Err(format!("hole '{name}': {left} of its bottom is not on {}", hole.under));
        }
        bodies.push(metal);
    }
    result.weld_joint_solids = joints;
    result.weld_source_seam = names.first().cloned();
    register(&mut result, &ctx.id, bodies, &colour);
    Ok(result)
}

// ------------------------------------------------------- spot and seam welds

/// The lap at a point of the top sheet's outer face: the two sheets'
/// thicknesses along the face's inward normal and the lower sheet's body.
struct Lap {
    upper: f64,
    lower: f64,
    under: String,
}

/// The nearest face of `handle`'s body straight across from `point` along
/// `down` whose outward normal is `facing`: its distance.
fn across(handle: u32, point: Vec3, down: Vec3, facing: Vec3, skip: Option<u64>) -> Result<Option<f64>, String> {
    crate::with_registered_solid_str(handle, |body| {
        let mut best: Option<f64> = None;
        for record in body.shells.iter().flat_map(|shell| &shell.faces) {
            if Some(record.id) == skip { continue; }
            let Ok((foot, outward)) = crate::fillet::face_plane_normal(record, point) else { continue };
            let reach = foot.sub(point);
            let distance = reach.dot(down);
            if distance <= 1e-9 || reach.sub(down.scale(distance)).length() > 1e-7 * (1.0 + distance) || outward.dot(facing) < 1.0 - 1e-9 {
                continue;
            }
            let projection = crate::project_point_to_surface(&record.surface, foot)?;
            let inside = crate::parameter_point_in_face(record, Vec2 { x: projection.u, y: projection.v }, 1e-7)?;
            if !matches!(inside, PolygonClass::Inside | PolygonClass::Boundary) { continue; }
            if best.is_none_or(|nearest| distance < nearest) { best = Some(distance); }
        }
        Ok(best)
    })
}

fn lap(ctx: &FeatureContext, plate: &str, face_id: u64, point: Vec3, normal: Vec3) -> Result<Lap, String> {
    let handle = ctx.scene.solids[plate];
    let down = normal.scale(-1.0);
    let upper = across(handle, point, down, down, Some(face_id))?
        .ok_or_else(|| format!("no face of {plate} straight under the weld line to measure its thickness"))?;
    let interface = point.add(down.scale(upper));
    let probe = interface.add(down.scale(1e-4));
    // The sheet the point lies in, else (where the line reaches the sheets'
    // edge) the first by name it lies on; several it lies in is ambiguous.
    let mut names: Vec<(&String, &u32)> = ctx.scene.solids.iter().collect();
    names.sort();
    let (mut inside, mut touching) = (Vec::new(), Vec::new());
    for (name, other) in names {
        if name == plate || super::fit::owned_by(name, &ctx.id) { continue; }
        match crate::with_registered_solid_str(*other, |body| Ok(crate::classify_point(probe, body, 1e-7)?.class))? {
            PointClass::In => inside.push(name.clone()),
            PointClass::On => touching.push(name.clone()),
            PointClass::Out => {}
        }
    }
    if inside.len() > 1 {
        return Err(format!("several sheets lie under {plate} at the weld line ({})", inside.join(", ")));
    }
    let under = inside.into_iter().next().or_else(|| touching.into_iter().next());
    let under = under.ok_or_else(|| format!("no sheet lies against {plate} under the weld line (at {:.6} {:.6} {:.6}, {upper} under the face)", interface.x, interface.y, interface.z))?;
    let lower = across(ctx.scene.solids[&under], interface, down, down, None)?
        .ok_or_else(|| format!("no face of {under} straight under the weld line to measure its thickness"))?;
    Ok(Lap { upper, lower, under })
}

/// The weld line and the top sheet's outer face it is laid from.
struct Line {
    chain: fit::Chain,
    plate: String,
    face_id: u64,
    normal: Vec3,
}

fn line(ctx: &FeatureContext) -> Result<Line, String> {
    let names = common::unique_reference_names(ctx.param("seam"));
    let mut paths = Vec::new();
    let mut faces = Vec::new();
    for name in &names {
        if !ctx.scene.resolve_face_group(name).is_empty() {
            faces.push(name.clone());
        } else {
            paths.push(name.clone());
        }
    }
    let [face] = faces.as_slice() else {
        return Err("select the weld line (edges or a sketch) and the top sheet's outer face".into());
    };
    if paths.is_empty() {
        return Err("select the weld line (edges or a sketch) on the top sheet's outer face".into());
    }
    let group = ctx.scene.resolve_face_group(face);
    let [reference] = group.as_slice() else { return Err(format!("face '{face}' stands for several faces; select one")) };
    let plate = body_of(ctx.scene, reference.handle).ok_or_else(|| format!("face '{face}' belongs to no resident part"))?;
    let curves = common::resolve_path_chain(ctx, &paths)?;
    let mut runs = Vec::new();
    for segment in &curves {
        runs.push(fit::Seam::from_curve(&segment.curve, None)?.ok_or("the weld line must be straight lines and circular arcs")?);
    }
    let chain = fit::Chain::new(runs)?;
    let first = chain.runs[0].start;
    let normal = crate::with_registered_solid_str(reference.handle, |body| {
        let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == reference.face_id)
            .ok_or_else(|| format!("face '{face}' is no longer resident"))?;
        let (foot, outward) = crate::fillet::face_plane_normal(record, first).map_err(|refusal| refusal.to_string())?;
        if foot.sub(first).length() > 1e-6 * (1.0 + first.length()) {
            return Err(format!("the weld line does not lie on face '{face}'"));
        }
        Ok(outward)
    })?;
    // The line lies on the face's plane all along.
    for run in &chain.runs {
        for fraction in [0.0, 0.5, 1.0] {
            if run.at(fraction).sub(first).dot(normal).abs() > 1e-6 * (1.0 + first.length()) {
                return Err(format!("the weld line does not lie in the plane of face '{face}'"));
            }
        }
    }
    Ok(Line { chain, plate, face_id: reference.face_id, normal })
}

/// The point `along` the chain (by length from its start) and the unit
/// tangent there.
fn station(chain: &fit::Chain, along: f64) -> (Vec3, Vec3) {
    let mut start = 0.0;
    for run in &chain.runs {
        if along <= start + run.length || std::ptr::eq(run, chain.runs.last().expect("a run")) {
            let fraction = ((along - start) / run.length).clamp(0.0, 1.0);
            return (run.at(fraction), run.carry(fraction, run.tangent));
        }
        start += run.length;
    }
    unreachable!("a chain has runs")
}

pub fn spot_context_applicable(probe: &SelectionProbe) -> bool {
    probe.faces == 1 && probe.edges + probe.sketches >= 1
}

pub fn spot_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD.SPOT", "shortName": "SPOT", "longName": "Spot weld",
        "ribbonPath": "Welding/Sheet metal and lap/Spot weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": with_common(serde_json::json!({
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE", "SKETCH", "FACE"], "multiple": true, "default_value": null,
                "hint": "Select the line the spots run along (edges or a sketch on the top sheet's outer face) and that face. The sheet under it is found through the lap."},
            "diameter": {"type": "number", "label": "Nugget diameter", "default_value": 5, "hint": "Diameter of each weld nugget, in model units."},
            "pitch": {"type": "number", "label": "Pitch", "default_value": 25, "hint": "Centre-to-centre distance of the spots along the line."},
            "endDistance": {"type": "number", "label": "End distance", "default_value": 10, "hint": "Distance of the first and last spot from the line's ends; the spots between are centred."},
            "inset": {"type": "number", "label": "Inset", "default_value": 0, "hint": "Move the spots this far off the line onto the face (for a line along the sheet's edge)."},
            "penetration": {"type": "number", "label": "Penetration", "default_value": 0.8, "hint": "How far the nugget reaches into each sheet, as a fraction of its thickness (less than 1)."},
            "marksOnly": {"type": "boolean", "label": "Marks only", "default_value": false, "hint": "Place the spots as marks (points and callout) without nugget solids."}
        }))
    })
}

pub fn execute_spot(ctx: &FeatureContext) -> FeatureResult {
    spot(ctx).unwrap_or_else(|error| ctx.fail(format!("spot weld: {error}")))
}

fn spot(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let diameter = number(ctx, "diameter", 5.0, "nugget diameter", true)?;
    let pitch = number(ctx, "pitch", 25.0, "pitch", true)?;
    let end = number(ctx, "endDistance", 10.0, "end distance", false)?;
    let inset = number(ctx, "inset", 0.0, "inset", false)?;
    let penetration = number(ctx, "penetration", 0.8, "penetration", true)?;
    if penetration >= 1.0 {
        return Err("penetration must be less than 1 (a nugget never reaches through a sheet)".into());
    }
    let marks = boolean(ctx, "marksOnly", false, "Marks only")?;
    let generate = boolean(ctx, "generateSolid", true, "Generate weld solid")? && !marks;
    let colour = colour(ctx)?;
    let line = line(ctx)?;
    let total = line.chain.length();
    if 2.0 * end > total + 1e-9 * total.max(1.0) {
        return Err(format!("the end distance {end} leaves no room on the {total} long line"));
    }
    let span = (total - 2.0 * end).max(0.0);
    let count = (span / pitch + 1e-9).floor() + 1.0;
    if count > super::MAX_SEGMENTS as f64 {
        return Err(format!("{count} spots on the {total} long line; at most {} (lengthen the pitch)", super::MAX_SEGMENTS));
    }
    let count = count as usize;
    let first = end + (span - (count - 1) as f64 * pitch) * 0.5;
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let plate_body = crate::registered_solid_clone(ctx.scene.solids[&line.plate])?;
    let mut bodies = Vec::new();
    let mut joints = vec![line.plate.clone()];
    for k in 0..count {
        let (on_line, tangent) = station(&line.chain, first + k as f64 * pitch);
        // Inset onto the face: across the line, to the side the face is on.
        let point = if inset > 0.0 {
            let across = line.normal.cross(tangent).normalized()?;
            let candidates = [on_line.add(across.scale(inset)), on_line.sub(across.scale(inset))];
            let mut chosen = None;
            for candidate in candidates {
                let probe = candidate.sub(line.normal.scale(1e-4));
                if crate::classify_point(probe, &plate_body, 1e-7)?.class == PointClass::In { chosen = Some(candidate); break; }
            }
            chosen.ok_or_else(|| format!("spot {}: inset {inset} lands off the sheet on both sides of the line", k + 1))?
        } else {
            on_line
        };
        result.points.push((format!("{}:Spot{}", ctx.id, k + 1), ScenePoint::model(point)));
        if k == 0 {
            result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(point)));
        }
        if !generate { continue; }
        let lap = lap(ctx, &line.plate, line.face_id, point, line.normal)?;
        if !joints.contains(&lap.under) { joints.push(lap.under.clone()); }
        // The nugget: centred on the faying surface, reaching `penetration`
        // of each sheet's thickness into it.
        let top = point.sub(line.normal.scale(lap.upper * (1.0 - penetration)));
        let height = penetration * (lap.upper + lap.lower);
        let nugget = crate::make_cylinder_brep(top, line.normal.scale(-1.0), diameter * 0.5, height)?;
        let under = crate::registered_solid_clone(ctx.scene.solids[&lap.under])?;
        let left = outside(&nugget, &[&plate_body, &under])?;
        if left > 1e-9 {
            return Err(format!("spot {}: {left} of the nugget lies outside the two sheets (too near an edge or a hole)", k + 1));
        }
        bodies.push(nugget);
    }
    result.weld_joint_solids = joints;
    result.weld_source_seam = common::unique_reference_names(ctx.param("seam")).first().cloned();
    let seam_curves: Vec<crate::NurbsCurve> = common::resolve_path_chain(ctx, &common::unique_reference_names(ctx.param("seam"))
        .into_iter().filter(|name| ctx.scene.resolve_face_group(name).is_empty()).collect::<Vec<_>>())?
        .into_iter().map(|segment| segment.curve).collect();
    result.paths.push((format!("{}:Seam", ctx.id), seam_curves));
    register(&mut result, &ctx.id, bodies, &colour);
    Ok(result)
}

pub fn seam_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD.RSEAM", "shortName": "RSEAM", "longName": "Seam weld",
        "ribbonPath": "Welding/Sheet metal and lap/Seam weld", "commandSize": "Compact", "displayBuilder": true,
        "inputParamsSchema": with_common(serde_json::json!({
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE", "SKETCH", "FACE"], "multiple": true, "default_value": null,
                "hint": "Select the line the seam runs along (edges or a sketch on the top sheet's outer face) and that face."},
            "width": {"type": "number", "label": "Width", "default_value": 5, "hint": "Width of the continuous nugget across the line, in model units."},
            "inset": {"type": "number", "label": "Inset", "default_value": 0, "hint": "Move a straight seam this far off the line onto the face (for a line along the sheet's edge)."},
            "penetration": {"type": "number", "label": "Penetration", "default_value": 0.8, "hint": "How far the nugget reaches into each sheet, as a fraction of its thickness (less than 1)."}
        }))
    })
}

pub fn execute_seam(ctx: &FeatureContext) -> FeatureResult {
    seam(ctx).unwrap_or_else(|error| ctx.fail(format!("seam weld: {error}")))
}

fn seam(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let width = number(ctx, "width", 5.0, "width", true)?;
    let penetration = number(ctx, "penetration", 0.8, "penetration", true)?;
    if penetration >= 1.0 {
        return Err("penetration must be less than 1 (a nugget never reaches through a sheet)".into());
    }
    let inset = number(ctx, "inset", 0.0, "inset", false)?;
    let generate = boolean(ctx, "generateSolid", true, "Generate weld solid")?;
    let colour = colour(ctx)?;
    let mut line = line(ctx)?;
    if inset > 0.0 {
        // A straight line moved across onto the face, to its side of the line.
        if !line.chain.is_single_line() {
            return Err("inset moves a single straight line; draw the seam's line on the face instead".into());
        }
        let run = &line.chain.runs[0];
        let across = line.normal.cross(run.tangent).normalized()?;
        let plate = crate::registered_solid_clone(ctx.scene.solids[&line.plate])?;
        let mut moved = None;
        for side in [1.0, -1.0] {
            let shift = across.scale(inset * side);
            let probe = run.at(0.5).add(shift).sub(line.normal.scale(1e-4));
            if crate::classify_point(probe, &plate, 1e-7)?.class == PointClass::In {
                moved = Some(fit::Seam::line(run.start.add(shift), run.tangent, run.length, None));
                break;
            }
        }
        let moved = moved.ok_or_else(|| format!("inset {inset} lands off the sheet on both sides of the line"))?;
        line.chain = fit::Chain::new(vec![moved])?;
    }
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let start = line.chain.runs[0].start;
    result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(line.chain.runs[0].at(0.5))));
    let lap = lap(ctx, &line.plate, line.face_id, start, line.normal)?;
    result.weld_joint_solids = vec![line.plate.clone(), lap.under.clone()];
    result.weld_source_seam = common::unique_reference_names(ctx.param("seam")).first().cloned();
    if generate {
        let trims = line.chain.trims(width * 0.5 + 100.0 * crate::tolerance::WELD_FLOOR)?;
        let (top, bottom) = (-lap.upper * (1.0 - penetration), -lap.upper - lap.lower * penetration);
        let mut strip: Option<BrepSolid> = None;
        for (run, trim) in line.chain.runs.iter().zip(&trims) {
            let across = line.normal.cross(run.tangent).normalized()?.scale(width * 0.5);
            let at = |side: f64, level: f64| run.start.add(across.scale(side)).add(line.normal.scale(level));
            let corners = [at(-1.0, top), at(1.0, top), at(1.0, bottom), at(-1.0, bottom)];
            let sides: Vec<Side> = (0..4).map(|i| Side::Line(corners[i], corners[(i + 1) % 4])).collect();
            let piece = fit::sweep_outline(run, &sides, trim)?;
            strip = Some(match strip {
                None => piece,
                Some(sum) => crate::boolean_operation(&sum, &piece, crate::BooleanOperation::Union, &crate::BooleanOptions::default())
                    .map_err(|refusal| format!("could not join the seam's runs: {refusal}"))?,
            });
        }
        let strip = strip.ok_or("the weld line is empty")?;
        let plate = crate::registered_solid_clone(ctx.scene.solids[&line.plate])?;
        let under = crate::registered_solid_clone(ctx.scene.solids[&lap.under])?;
        let left = outside(&strip, &[&plate, &under])?;
        if left > 1e-9 {
            return Err(format!("{left} of the seam lies outside the two sheets (too near an edge, a hole, or the sheets' thickness changes along the line)"));
        }
        // The strip is one section along the whole line, so the sheets must
        // keep their thicknesses all along it, checked exactly under the
        // strip's width: each sheet fills its band, and neither runs on past
        // the top face or the lower sheet's bottom.
        let skin = 100.0 * crate::tolerance::WELD_FLOOR;
        let band = |from: f64, to: f64| -> Result<BrepSolid, String> {
            let mut solid: Option<BrepSolid> = None;
            for (run, trim) in line.chain.runs.iter().zip(&trims) {
                let across = line.normal.cross(run.tangent).normalized()?.scale(width * 0.5);
                let at = |side: f64, level: f64| run.start.add(across.scale(side)).add(line.normal.scale(level));
                let corners = [at(-1.0, from), at(1.0, from), at(1.0, to), at(-1.0, to)];
                let sides: Vec<Side> = (0..4).map(|i| Side::Line(corners[i], corners[(i + 1) % 4])).collect();
                let piece = fit::sweep_outline(run, &sides, trim)?;
                solid = Some(match solid {
                    None => piece,
                    Some(sum) => crate::boolean_operation(&sum, &piece, crate::BooleanOperation::Union, &crate::BooleanOptions::default())
                        .map_err(|refusal| format!("could not join the seam's runs: {refusal}"))?,
                });
            }
            solid.ok_or_else(|| "the weld line is empty".to_string())
        };
        let changed = |what: &str| format!("the sheets change along the line ({what}; {} + {} thick at its start); weld each stretch as its own seam", lap.upper, lap.lower);
        // Thin slabs on each sheet's two faces under the strip, as the fillet's
        // support slabs are: just inside a face, each lies wholly in its sheet
        // (no sheet thins, ends or opens there), and just outside the top face
        // and the lower sheet's bottom, none touches it (no sheet thickens).
        let (u, l) = (lap.upper, lap.lower);
        let inside = [
            (0.0, -skin, &plate, "the top sheet's face drops or opens"),
            (-u + skin, -u, &plate, "the top sheet thins"),
            (-u, -u - skin, &under, "the lower sheet's face drops or opens"),
            (-u - l + skin, -u - l, &under, "the lower sheet thins"),
        ];
        for (from, to, sheet, what) in inside {
            if outside(&band(from, to)?, &[sheet])? > 1e-9 { return Err(changed(what)); }
        }
        if fit::common_volume(&band(skin, 0.0)?, &plate)? > 1e-9 { return Err(changed("the top sheet thickens")); }
        if fit::common_volume(&band(-u - l, -u - l - skin)?, &under)? > 1e-9 { return Err(changed("the lower sheet thickens")); }
        register(&mut result, &ctx.id, vec![strip], &colour);
    }
    let curves: Vec<crate::NurbsCurve> = common::resolve_path_chain(ctx, &common::unique_reference_names(ctx.param("seam"))
        .into_iter().filter(|name| ctx.scene.resolve_face_group(name).is_empty()).collect::<Vec<_>>())?
        .into_iter().map(|segment| segment.curve).collect();
    result.paths.push((format!("{}:Seam", ctx.id), curves));
    Ok(result)
}
