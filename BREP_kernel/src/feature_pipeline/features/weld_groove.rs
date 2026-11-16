//! Groove (butt) welds: the bead fills the joint two butting parts' faces
//! make, with an optional cap on the face the groove opens to and a root bead
//! on the other. In a part, the joint preparation (bevel, J or U) can be cut
//! into the parts by the weld; an assembly weld never changes its components.
use super::common;
use super::fit::{self, Seam, Side, Trim};

#[path = "weld_edge.rs"]
pub(crate) mod edge;
use crate::feature_pipeline::{FeatureContext, FeatureResult, ScenePoint, SelectionProbe};
use crate::Vec3;

/// The groove's shape: how each butting face is prepared.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Shape {
    /// Both faces square: the bead is the root opening and its caps.
    Square,
    /// Both faces bevelled, each by half the groove angle.
    V,
    /// One face bevelled by the groove angle, the other square.
    Bevel,
    /// Both faces J-shaped: a root radius, then a bevel of half the angle.
    U,
    /// One face J-shaped by the groove angle, the other square.
    J,
}

const SHAPES: [(&str, Shape); 5] = [("SQUARE", Shape::Square), ("V", Shape::V), ("BEVEL", Shape::Bevel), ("U", Shape::U), ("J", Shape::J)];

/// How one face of the joint is prepared.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Edge {
    Square,
    /// A straight bevel at this angle from square, in radians.
    Bevel(f64),
    /// A root radius blending into a bevel at this angle, in radians.
    J(f64, f64),
}

/// The schema fields the groove weld's dialog hides for these values.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    let shape = values.get("shape").and_then(serde_json::Value::as_str).unwrap_or("V");
    let mut hidden = Vec::new();
    if shape == "SQUARE" {
        hidden.extend(["grooveAngle", "rootFace", "prepare"]);
    }
    if !matches!(shape, "U" | "J") {
        hidden.push("rootRadius");
    }
    if !matches!(shape, "BEVEL" | "J") {
        hidden.push("bevelFace");
    }
    if values.get("double").and_then(serde_json::Value::as_bool).unwrap_or(false) {
        hidden.push("rootReinforcement");
    }
    hidden
}

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.faces >= 2
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD.GROOVE", "shortName": "GROOVE", "longName": "Groove weld",
        "ribbonPath": "Welding/Fillet and groove/Groove weld", "commandSize": "Large", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["FACE", "EDGE"], "multiple": true, "default_value": null,
                "hint": "Select the two butting faces, in order (the first is the first joint face). Add an edge of either face to set the seam's direction and the side the groove opens to; without one the seam runs along the faces' longer side and the groove opens upwards (+Z, else +Y, else +X)."},
            "shape": {"type": "options", "label": "Groove", "options": ["SQUARE", "V", "BEVEL", "U", "J"], "default_value": "V",
                "hint": "SQUARE: both faces square, the root opening filled. V and U: both faces prepared. BEVEL and J: one face prepared (Bevelled face), the other square."},
            "double": {"type": "boolean", "label": "Double", "default_value": false,
                "hint": "Prepare and weld from both sides: the groove is mirrored about the middle of the thickness, a cap on each side."},
            "grooveAngle": {"type": "number", "label": "Groove angle", "default_value": 60,
                "hint": "The groove's included angle in degrees: split between the two faces for V and U, all on the bevelled face for BEVEL and J."},
            "rootFace": {"type": "number", "label": "Root face", "default_value": 0,
                "hint": "The unprepared land at the root, in model units (for a double groove, in the middle). The groove depth is the thickness less the root face."},
            "rootRadius": {"type": "number", "label": "Root radius", "default_value": 3,
                "hint": "The radius at the bottom of a U or J groove, in model units."},
            "bevelFace": {"type": "options", "label": "Bevelled face", "options": ["FIRST", "SECOND"], "default_value": "FIRST",
                "hint": "Which joint face a BEVEL or J groove prepares."},
            "reinforcement": {"type": "number", "label": "Reinforcement", "default_value": 1,
                "hint": "Height of the cap above the face the groove opens to (both faces for a double groove), in model units. 0 = flush."},
            "rootReinforcement": {"type": "number", "label": "Root reinforcement", "default_value": 0,
                "hint": "Height of the root bead below the other face, in model units. 0 = none."},
            "capOverlap": {"type": "number", "label": "Cap overlap", "default_value": 1,
                "hint": "How far a cap or root bead runs onto each part's face past the groove's edge, in model units."},
            "prepare": {"type": "boolean", "label": "Prepare joint", "default_value": true,
                "hint": "Cut the groove's bevel, J or U into the parts (a part weld). Off: the parts must already be prepared; an assembly weld never changes its components."},
            "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
                "hint": "On by default: build the filled groove as a separate solid. Off keeps the weld, its preparation and its callout."},
            "weldColor": {"type": "color", "label": "Weld colour", "default_value": "#d9822b",
                "hint": "Colour of the bead solid in the 3D view and exports."},
            "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
        }
    })
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    build(ctx).unwrap_or_else(|error| ctx.fail(format!("groove weld: {error}")))
}

/// The joint two butting flat faces make: where its section's origin is and
/// how its axes run. `u` points from the first face across to the second,
/// `w` from the back face to the face the groove opens to, `d` along the seam.
struct Joint {
    faces: [JointFace; 2],
    origin: Vec3,
    u: Vec3,
    w: Vec3,
    d: Vec3,
    gap: f64,
    thickness: f64,
    length: f64,
    /// Whether each part ends at the seam's start and at its end (and so the
    /// preparation may run out past it there).
    ends: [[bool; 2]; 2],
}

struct JointFace {
    name: String,
    solid: String,
}

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

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let shape = match ctx.param("shape") {
        None => Shape::V,
        Some(serde_json::Value::String(value)) => SHAPES.iter().find(|(name, _)| *name == value.as_str()).map(|(_, shape)| *shape)
            .ok_or_else(|| format!("unknown groove '{value}' (SQUARE, V, BEVEL, U or J)"))?,
        Some(_) => return Err("groove must be SQUARE, V, BEVEL, U or J".into()),
    };
    let double = boolean(ctx, "double", false, "Double")?;
    let angle = number(ctx, "grooveAngle", 60.0, "groove angle", true)?;
    // Each prepared face leans at most 80 degrees from square, so every part
    // corner along the groove keeps at least 10 degrees of metal.
    let limit = if matches!(shape, Shape::Bevel | Shape::J) { 80.0 } else { 160.0 };
    if shape != Shape::Square && angle >= limit {
        return Err(format!("groove angle must be less than {limit} degrees for this groove"));
    }
    let angle = angle.to_radians();
    let root_face = number(ctx, "rootFace", 0.0, "root face", false)?;
    let radius = number(ctx, "rootRadius", 3.0, "root radius", true)?;
    let first_bevelled = match ctx.param("bevelFace") {
        None => true,
        Some(serde_json::Value::String(value)) if value == "FIRST" => true,
        Some(serde_json::Value::String(value)) if value == "SECOND" => false,
        Some(_) => return Err("bevelled face must be FIRST or SECOND".into()),
    };
    let cap = number(ctx, "reinforcement", 1.0, "reinforcement", false)?;
    let root_bead = if double { 0.0 } else { number(ctx, "rootReinforcement", 0.0, "root reinforcement", false)? };
    let overlap = number(ctx, "capOverlap", 1.0, "cap overlap", false)?;
    let prepare = shape != Shape::Square && boolean(ctx, "prepare", true, "Prepare joint")?;
    let generate = boolean(ctx, "generateSolid", true, "Generate weld solid")?;
    let colour = match ctx.param("weldColor") {
        None => super::WELD_COLOUR.to_string(),
        Some(serde_json::Value::String(hex)) if super::is_hex_colour(hex) => hex.to_ascii_lowercase(),
        Some(_) => return Err("weld colour must be `#rrggbb`".into()),
    };
    let joint = joint(ctx)?;
    let t = joint.thickness;
    let tolerance = 1e-6_f64.max(joint.length.max(t) * 1e-12);
    if matches!(shape, Shape::U | Shape::J) && root_face <= tolerance {
        return Err("a U or J groove needs a root face: its radius would run out along the back face to a feather edge".into());
    }
    if root_face >= t - tolerance && shape != Shape::Square {
        return Err(format!("the root face {root_face} leaves no groove in the {t} thick joint"));
    }
    // How each face is prepared (the first face is x ≤ 0, the second x ≥ gap).
    let prepared = |bevelled: bool, share: f64| -> Edge {
        match (shape, bevelled) {
            (Shape::Square, _) | (_, false) => Edge::Square,
            (Shape::V | Shape::Bevel, true) => Edge::Bevel(angle * share),
            (Shape::U | Shape::J, true) => Edge::J(radius, angle * share),
        }
    };
    let edges = match shape {
        Shape::Square => [Edge::Square, Edge::Square],
        Shape::V | Shape::U => [prepared(true, 0.5), prepared(true, 0.5)],
        Shape::Bevel | Shape::J => [prepared(first_bevelled, 1.0), prepared(!first_bevelled, 1.0)],
    };
    // The groove's depth from each face it opens to, and where its root starts.
    let depth = if double { (t - root_face) * 0.5 } else { t - root_face };
    let section = Section { edges, gap: joint.gap, thickness: t, depth, double, tolerance };
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let seam = Seam::line(joint.origin, joint.d, joint.length, Some(joint.faces[0].solid.clone()));
    let at = |x: f64, y: f64| joint.origin.add(joint.u.scale(x)).add(joint.w.scale(y));
    // The parts as the weld leaves them: prepared, or as they were.
    let mut parts: Vec<(String, crate::BrepSolid)> = Vec::new();
    for face in &joint.faces {
        let handle = ctx.scene.solids.get(&face.solid).copied().ok_or_else(|| format!("part '{}' is not resident", face.solid))?;
        parts.push((face.solid.clone(), crate::registered_solid_clone(handle)?));
    }
    let mut cut = [false; 2];
    if prepare {
        if !ctx.scene.components.is_empty() {
            return Err("Prepare joint cuts the parts, and an assembly weld never changes its components: prepare the parts in their own documents and turn Prepare joint off".into());
        }
        // A cutter for each prepared face, past the faces it opens to and
        // past the seam's ends, reaching just across the face's plane (into
        // the root opening, never into the other part, which is not cut).
        let margin = t.max(joint.length) * 0.05 + 1.0;
        for (side, (name, body)) in parts.iter_mut().enumerate() {
            if section.edges[side] == Edge::Square { continue; }
            // Along the seam the cut runs out past an end only where the part
            // ends there too; a part that runs on keeps its square face past
            // the joint.
            let [before, after] = joint.ends[side].map(|ends| if ends { margin } else { 0.0 });
            let cut_seam = Seam::line(joint.origin.sub(joint.d.scale(before)), joint.d, joint.length + before + after, None);
            let back = |x: f64, y: f64| at(x, y).sub(joint.d.scale(before));
            let outline: Vec<Side> = section.cutter(side, margin)?.into_iter().map(|side| side.place(&back)).collect();
            let mut cutter = fit::sweep_outline(&cut_seam, &outline, &Trim::default())?;
            // The faces the cut leaves on the part are named for the weld
            // and the side, so the two prepared parts never share a name.
            for (i, face) in cutter.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
                face.name = Some(format!("{}:Prep{}Face{}", ctx.id, side + 1, i + 1));
            }
            *body = common::subtract_solid(body.clone(), cutter)
                .map_err(|refusal| format!("could not prepare {name}: {refusal}"))?;
            cut[side] = true;
        }
    }
    let mut beads: Vec<(String, crate::BrepSolid)> = Vec::new();
    if generate {
        let regions = section.regions(cap, root_bead, overlap)?;
        if regions.is_empty() {
            return Err("a square groove with no root opening and no reinforcement has nothing to fill".into());
        }
        for (n, region) in regions.iter().enumerate() {
            let name = if n == 0 { ctx.id.clone() } else { format!("{}:Bead{}", ctx.id, n + 1) };
            let outline: Vec<Side> = region.outline.iter().map(|side| side.place(&at)).collect();
            let mut bead = fit::sweep_outline(&seam, &outline, &Trim::default())?;
            for (i, face) in bead.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
                face.name = Some(format!("{name}:BeadFace{}", i + 1));
            }
            // Exact checks: clear of every body (the prepared parts as cut),
            // and every side the bead lays against a part backed by it.
            let replaced: Vec<(&str, &crate::BrepSolid)> = parts.iter().map(|(name, body)| (name.as_str(), body)).collect();
            if let Err(reason) = fit::clear_of(ctx.scene, &bead, &ctx.id, &replaced, tolerance)? {
                return Err(if !prepare && shape != Shape::Square && reason.contains("would overlap") {
                    format!("{reason}: the parts are not prepared for this groove; turn on Prepare joint (a part weld) or cut the groove into them first")
                } else {
                    reason
                });
            }
            for (side, contact) in &region.contacts {
                let body = &parts[*side].1;
                // The part's side of the contact, read halfway along the seam.
                let inside = |q: [f64; 2]| -> Result<bool, String> {
                    let point = at(q[0], q[1]).add(joint.d.scale(joint.length * 0.5));
                    Ok(crate::classify_point(point, body, slab_depth() * 1e-3)?.class == crate::PointClass::In)
                };
                let unbacked = || format!("the bead is not backed by {} along the whole joint{}", joint.faces[*side].name,
                    if !prepare && shape != Shape::Square { " (are the parts prepared for this groove?)" } else { "" });
                let Some(outline) = slab(contact, &inside)? else { return Err(unbacked()) };
                let outline: Vec<Side> = outline.iter().map(|side| side.place(&at)).collect();
                let slab = fit::sweep_outline(&seam, &outline, &Trim::default())?;
                if let Err(reason) = fit::backed(&slab, body)? {
                    return Err(format!("{reason}: {}", unbacked()));
                }
            }
            beads.push((name, bead));
        }
    }
    // The prepared parts replace the parts they were cut from.
    for ((name, body), cut) in parts.into_iter().zip(cut) {
        if cut {
            result.added.push(common::register_added(body, &name));
            result.removed.push(name);
        }
    }
    for (name, bead) in beads {
        result.added.push(common::register_added(bead, &name));
        super::paint_bead(&name, &colour);
    }
    for face in &joint.faces {
        if !result.weld_joint_solids.contains(&face.solid) {
            result.weld_joint_solids.push(face.solid.clone());
        }
    }
    // The seam along the middle of the groove's opening.
    let top = at(joint.gap * 0.5, t);
    let seam_curve = crate::make_line(top, top.add(joint.d.scale(joint.length)))?;
    result.weld_source_seam = Some(joint.faces[0].name.clone());
    result.paths.push((format!("{}:Seam", ctx.id), vec![seam_curve]));
    result.points.push((format!("{}:Callout", ctx.id), ScenePoint::model(top.add(joint.d.scale(joint.length * 0.5)))));
    Ok(result)
}

/// Read the joint from the picks: two flat faces of two parts, parallel and
/// facing each other across the root opening, the same height across the
/// joint; an optional edge of either face sets the seam's direction and the
/// side the groove opens to.
fn joint(ctx: &FeatureContext) -> Result<Joint, String> {
    let names = common::unique_reference_names(ctx.param("seam"));
    let mut faces: Vec<String> = Vec::new();
    let mut edge: Option<String> = None;
    for name in &names {
        if !ctx.scene.resolve_face_group(name).is_empty() {
            faces.push(name.clone());
        } else if ctx.scene.resolve_edge(name).is_some() {
            if edge.replace(name.clone()).is_some() {
                return Err("select at most one edge (the seam's direction and the side the groove opens to)".into());
            }
        } else {
            return Err(format!("reference '{name}' is not a resident face or edge"));
        }
    }
    let [first, second] = faces.as_slice() else {
        return Err("select the two butting faces".into());
    };
    let flat = |name: &str| -> Result<(String, u32, Vec3, Vec3, Vec<Vec3>, Vec<(Vec3, Vec3)>), String> {
        let group = ctx.scene.resolve_face_group(name);
        let [face] = group.as_slice() else { return Err(format!("face '{name}' stands for several faces; select one")) };
        let solid = ctx.scene.solids.iter().filter(|(_, handle)| **handle == face.handle)
            .map(|(name, _)| name.clone()).min().ok_or_else(|| format!("face '{name}' belongs to no resident part"))?;
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
            let (on, outward) = crate::fillet::face_plane_normal(record, first).map_err(|refusal| refusal.to_string())?;
            let size = corners.iter().map(|p| p.sub(first).length()).fold(0.0, f64::max).max(1.0);
            let level = record.surface.control_points.iter().flatten().all(|pole| {
                let w = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
                Vec3::new(pole.x / w, pole.y / w, pole.z / w).sub(on).dot(outward).abs() <= 1e-9 * size
            });
            if !level {
                return Err(format!("face '{name}' is not flat; a groove weld joins two flat butting faces"));
            }
            Ok((solid, face.handle, on, outward, corners, lines))
        })
    };
    let (solid_a, handle_a, on_a, normal_a, corners_a, lines_a) = flat(first)?;
    let (solid_b, handle_b, on_b, normal_b, corners_b, _) = flat(second)?;
    if handle_a == handle_b {
        return Err("the two faces belong to one part; a groove weld joins two parts".into());
    }
    if normal_a.dot(normal_b) > -1.0 + 1e-9 {
        return Err(format!("faces '{first}' and '{second}' do not face each other (parallel and opposed)"));
    }
    let u = normal_a;
    let gap = on_b.sub(on_a).dot(u);
    let size = corners_a.iter().chain(&corners_b).map(|p| p.sub(on_a).length()).fold(1.0, f64::max);
    let tolerance = 1e-9 * size;
    if gap < -tolerance {
        return Err(format!("faces '{first}' and '{second}' overlap by {}", -gap));
    }
    let gap = gap.max(0.0);
    // The seam's direction: the picked edge's, else the first face's longer side.
    let mut directions: Vec<Vec3> = Vec::new();
    for (a, b) in &lines_a {
        let Ok(dir) = b.sub(*a).normalized() else { continue };
        if !directions.iter().any(|known| known.cross(dir).length() < 1e-9) { directions.push(dir); }
    }
    let extent = |dir: Vec3, points: &[Vec3]| -> (f64, f64) {
        points.iter().map(|p| p.sub(on_a).dot(dir)).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| (lo.min(s), hi.max(s)))
    };
    let (d, w) = if let Some(edge_name) = &edge {
        let reference = ctx.scene.resolve_edge(edge_name).expect("resolved above");
        let curve = common::edge_curve(reference)?;
        let [t0, t1] = curve.domain()?;
        let (a, b) = (curve.evaluate(t0)?, curve.evaluate(t1)?);
        let d = b.sub(a).normalized()?;
        if d.dot(u).abs() > 1e-9 {
            return Err(format!("edge '{edge_name}' does not run along the faces"));
        }
        let w = u.cross(d);
        // The groove opens to the side of the faces the edge is on.
        let middle = a.add(b).scale(0.5).sub(on_a).dot(w);
        let (low, high) = extent(w, &corners_a);
        if (middle - low).abs() > tolerance && (middle - high).abs() > tolerance {
            return Err(format!("edge '{edge_name}' is not on the side of the faces the groove opens to"));
        }
        (d, if (middle - high).abs() <= tolerance { w } else { w.scale(-1.0) })
    } else {
        let d = directions.iter().copied().max_by(|a, b| {
            let (la, ha) = extent(*a, &corners_a);
            let (lb, hb) = extent(*b, &corners_a);
            (ha - la).total_cmp(&(hb - lb))
        }).ok_or_else(|| format!("face '{first}' has no straight edge to run the seam along"))?;
        let w = u.cross(d).normalized()?;
        // Upwards: +Z, else +Y, else +X.
        let up = [w.z, w.y, w.x].into_iter().find(|c| c.abs() > 1e-9).unwrap_or(1.0);
        (d, if up > 0.0 { w } else { w.scale(-1.0) })
    };
    // The joint's thickness is the parts', between their faces square to w
    // nearest above and below the picked faces: a picked face may be an
    // already prepared part's root face (its land), shorter than the plate.
    let levels = |handle: u32, low: f64, high: f64| -> Result<Option<(f64, f64)>, String> {
        crate::with_registered_solid_str(handle, |body| {
            let (mut bottom, mut top): (Option<f64>, Option<f64>) = (None, None);
            for record in body.shells.iter().flat_map(|shell| &shell.faces) {
                let Some(pole) = record.surface.control_points.iter().flatten().next() else { continue };
                let weight = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
                let near = Vec3::new(pole.x / weight, pole.y / weight, pole.z / weight);
                let Ok((on, outward)) = crate::fillet::face_plane_normal(record, near) else { continue };
                let level_face = record.surface.control_points.iter().flatten().all(|pole| {
                    let weight = if pole.w.abs() > 1e-300 { pole.w } else { 1.0 };
                    Vec3::new(pole.x / weight, pole.y / weight, pole.z / weight).sub(on).dot(outward).abs() <= tolerance
                });
                if !level_face { continue; }
                let level = on.sub(on_a).dot(w);
                if outward.dot(w) > 1.0 - 1e-9 && level >= high - tolerance && top.is_none_or(|t| level < t) { top = Some(level); }
                if outward.dot(w) < -1.0 + 1e-9 && level <= low + tolerance && bottom.is_none_or(|b| level > b) { bottom = Some(level); }
            }
            Ok(bottom.zip(top))
        })
    };
    let (aw0, aw1) = extent(w, &corners_a);
    let (bw0, bw1) = extent(w, &corners_b);
    let ((pa0, pa1), (pb0, pb1)) = match (levels(handle_a, aw0, aw1)?, levels(handle_b, bw0, bw1)?) {
        (Some(a), Some(b)) => (a, b),
        _ => return Err(format!("faces '{first}' and '{second}' do not run between two flat faces of their parts across the joint")),
    };
    if (pa0 - pb0).abs() > tolerance * 1e3 || (pa1 - pb1).abs() > tolerance * 1e3 {
        return Err(format!("the parts of '{first}' ({}) and '{second}' ({}) are not the same thickness across the joint; a groove weld between unequal parts is not built",
            pa1 - pa0, pb1 - pb0));
    }
    let (aw0, aw1) = (pa0, pa1);
    let (ad0, ad1) = extent(d, &corners_a);
    let (bd0, bd1) = extent(d, &corners_b);
    let (d0, d1) = (ad0.max(bd0), ad1.min(bd1));
    if d1 - d0 <= tolerance {
        return Err(format!("faces '{first}' and '{second}' do not overlap along the seam"));
    }
    Ok(Joint {
        faces: [JointFace { name: first.clone(), solid: solid_a }, JointFace { name: second.clone(), solid: solid_b }],
        origin: on_a.add(d.scale(d0)).add(w.scale(aw0)),
        u, w, d, gap, thickness: aw1 - aw0, length: d1 - d0,
        ends: [[ad0 >= d0 - tolerance * 1e3, ad1 <= d1 + tolerance * 1e3], [bd0 >= d0 - tolerance * 1e3, bd1 <= d1 + tolerance * 1e3]],
    })
}

/// A side of the section in its own plane: x across the joint (0 on the
/// first face, `gap` on the second), y through the thickness (0 on the back
/// face, `thickness` on the face the groove opens to).
#[derive(Clone, Copy, Debug)]
enum Flat {
    Line([f64; 2], [f64; 2]),
    Arc { from: [f64; 2], to: [f64; 2], centre: [f64; 2], through: [f64; 2] },
}

impl Flat {
    fn place(&self, at: &impl Fn(f64, f64) -> Vec3) -> Side {
        let p = |q: [f64; 2]| at(q[0], q[1]);
        match *self {
            Flat::Line(a, b) => Side::Line(p(a), p(b)),
            Flat::Arc { from, to, centre, through } => Side::Arc { from: p(from), to: p(to), centre: p(centre), through: p(through) },
        }
    }

    fn reversed(&self) -> Flat {
        match *self {
            Flat::Line(a, b) => Flat::Line(b, a),
            Flat::Arc { from, to, centre, through } => Flat::Arc { from: to, to: from, centre, through },
        }
    }

    fn end(&self) -> [f64; 2] {
        match *self { Flat::Line(_, b) => b, Flat::Arc { to, .. } => to }
    }
}

/// One body of the bead: its closed outline, and the sides of it that lie
/// against a part (which part, and the side).
struct Region {
    outline: Vec<Flat>,
    contacts: Vec<(usize, Flat)>,
}

/// The groove's cross-section.
struct Section {
    edges: [Edge; 2],
    gap: f64,
    thickness: f64,
    /// The groove's depth from each face it opens to.
    depth: f64,
    double: bool,
    tolerance: f64,
}

/// The material depth the support slabs read under each contact.
fn slab_depth() -> f64 {
    100.0 * crate::tolerance::WELD_FLOOR
}

impl Section {
    /// Where the groove of face `side` starts (its root) and its profile up
    /// to the face it opens to, as sides from the root up, in the first
    /// face's frame (x ≤ 0 is into the part). `base` is the root's height,
    /// `top` the opening face's.
    fn flank(&self, side: usize, base: f64, top: f64) -> Result<Vec<Flat>, String> {
        let rise = top - base;
        Ok(match self.edges[side] {
            Edge::Square => vec![Flat::Line([0.0, base], [0.0, top])],
            Edge::Bevel(angle) => vec![Flat::Line([0.0, base], [-rise * angle.tan(), top])],
            Edge::J(radius, angle) => {
                // The radius sits on the root, centred over it; the bevel
                // leaves it where its tangent leans `angle` from square.
                let centre = [0.0, base + radius];
                let end = [-radius * angle.cos(), base + radius - radius * angle.sin()];
                if end[1] > top - self.tolerance {
                    return Err(format!("the {radius} root radius does not fit the {rise} deep groove"));
                }
                let half = (std::f64::consts::FRAC_PI_2 - angle) * 0.5;
                let through = [-radius * half.sin(), base + radius - radius * half.cos()];
                let rest = top - end[1];
                vec![
                    Flat::Arc { from: [0.0, base], to: end, centre, through },
                    Flat::Line(end, [end[0] - rest * angle.tan(), top]),
                ]
            }
        })
    }

    /// The boundary of face `side`'s part along the groove from the back
    /// face (y = 0) to the opening face (y = thickness), in the section's
    /// frame: the first face's as is, the second's mirrored across the gap.
    fn boundary(&self, side: usize) -> Result<Vec<Flat>, String> {
        let t = self.thickness;
        let land = t - if self.double { 2.0 * self.depth } else { self.depth };
        let mut sides = Vec::new();
        if self.double {
            // The lower groove: the upper one's profile mirrored through the
            // middle, walked from the back face up to its root.
            let base = (t - land) * 0.5;
            let lower: Vec<Flat> = self.flank(side, base + land, t)?.iter().rev()
                .map(|flat| mirror_y(&flat.reversed(), t)).collect();
            sides.extend(lower);
            if land > self.tolerance {
                sides.push(Flat::Line([0.0, base], [0.0, base + land]));
            }
            sides.extend(self.flank(side, base + land, t)?);
        } else {
            if land > self.tolerance {
                sides.push(Flat::Line([0.0, 0.0], [0.0, land]));
            }
            sides.extend(self.flank(side, land, t)?);
        }
        Ok(if side == 0 { sides } else { sides.iter().map(|flat| mirror_x(flat, self.gap)).collect() })
    }

    /// The bead's bodies: the filled groove with its caps when the root
    /// opening joins them into one, else each groove and cap on its own.
    fn regions(&self, cap: f64, root_bead: f64, overlap: f64) -> Result<Vec<Region>, String> {
        let t = self.thickness;
        let left = self.boundary(0)?;
        let right = self.boundary(1)?;
        let bottom_cap = if self.double { cap } else { root_bead };
        let mut regions = Vec::new();
        if self.gap > self.tolerance {
            // One region: up the second face's boundary, across the top,
            // down the first's, across the bottom.
            let (l1, r1) = (end(&left), end(&right));
            let (l0, r0) = (start(&left), start(&right));
            let mut outline: Vec<Flat> = right.clone();
            outline.extend(cap_sides(r1, l1, t, cap, overlap, 1.0));
            outline.extend(left.iter().rev().map(Flat::reversed));
            outline.extend(cap_sides(l0, r0, 0.0, bottom_cap, overlap, -1.0));
            let mut contacts: Vec<(usize, Flat)> = left.iter().map(|flat| (0, *flat)).chain(right.iter().map(|flat| (1, *flat))).collect();
            contacts.extend(toes(l1, r1, cap, overlap));
            contacts.extend(toes(l0, r0, bottom_cap, overlap));
            regions.push(Region { outline, contacts });
            return Ok(regions);
        }
        // No root opening: the faces meet at the root. Each groove opening to
        // a face is a region of its own (with that face's cap); a face with
        // no groove there has only its cap.
        for (opening, height, sign) in [(t, cap, 1.0), (0.0, bottom_cap, -1.0)] {
            // This face's groove sides on each part, from the root out to the face.
            let groove = |sides: &[Flat]| -> Vec<Flat> {
                let picked: Vec<Flat> = sides.iter().copied().filter(|flat| {
                    let ([x0, y0], [x1, y1]) = (start(std::slice::from_ref(flat)), flat.end());
                    let on_plane = x0.abs() <= self.tolerance && x1.abs() <= self.tolerance;
                    let mid = (y0 + y1) * 0.5;
                    !on_plane && if self.double { (mid > t * 0.5) == (sign > 0.0) } else { sign > 0.0 }
                }).collect();
                if sign > 0.0 { picked } else { picked.iter().rev().map(Flat::reversed).collect() }
            };
            let (l, r) = (groove(&left), groove(&right));
            if l.is_empty() && r.is_empty() {
                // Square at this face (or the back of a single groove): a cap alone.
                if height > 0.0 && overlap > 0.0 {
                    let (a, b) = ([overlap, opening], [-overlap, opening]);
                    let outline = vec![arc_flat(a, b, height * sign), Flat::Line(b, a)];
                    let contacts = vec![(0, Flat::Line(b, [0.0, opening])), (1, Flat::Line([0.0, opening], a))];
                    regions.push(Region { outline, contacts });
                }
                continue;
            }
            // The groove from its root (where the two faces meet) to this face;
            // a square face is the plane from the root.
            let root = if l.is_empty() { start(&r) } else { start(&l) };
            let square = vec![Flat::Line(root, [0.0, opening])];
            let l = if l.is_empty() { square.clone() } else { l };
            let r = if r.is_empty() { square } else { r };
            let (l1, r1) = (end(&l), end(&r));
            let mut outline: Vec<Flat> = r.clone();
            outline.extend(cap_sides(r1, l1, opening, height, overlap, sign));
            outline.extend(l.iter().rev().map(Flat::reversed));
            let mut contacts: Vec<(usize, Flat)> = l.iter().map(|flat| (0, *flat)).chain(r.iter().map(|flat| (1, *flat))).collect();
            contacts.extend(toes(l1, r1, height, overlap));
            regions.push(Region { outline, contacts });
        }
        Ok(regions)
    }

    /// The cutter that prepares face `side`'s part: the groove on that side
    /// of the joint plane, carried `margin` past each face it opens to and a
    /// hair across the plane into the root opening (the other part is not cut).
    fn cutter(&self, side: usize, margin: f64) -> Result<Vec<Flat>, String> {
        let t = self.thickness;
        let sign = if side == 0 { 1.0 } else { -1.0 };
        let plane = if side == 0 { 0.0 } else { self.gap };
        let across = plane + sign * (self.gap * 0.5).min(slab_depth()).max(slab_depth() * 0.5);
        let boundary = self.boundary(side)?;
        // The groove sides: off the joint plane.
        let off = |flat: &Flat| {
            let [x0, _] = start(std::slice::from_ref(flat));
            let [x1, _] = flat.end();
            !((x0 - plane).abs() <= self.tolerance && (x1 - plane).abs() <= self.tolerance)
        };
        let mut cutters = Vec::new();
        for upper in [true, false] {
            if !upper && !self.double { continue; }
            let sides: Vec<Flat> = boundary.iter().copied().filter(|flat| {
                let mid = (start(std::slice::from_ref(flat))[1] + flat.end()[1]) * 0.5;
                off(flat) && (if upper { mid > t * 0.5 || !self.double } else { mid < t * 0.5 })
            }).collect();
            if sides.is_empty() { continue; }
            // Walk from the root out to the face, then past it along the last side.
            let sides: Vec<Flat> = if upper { sides } else { sides.iter().rev().map(Flat::reversed).collect() };
            let root = start(&sides);
            let face = sides.last().expect("sides").end();
            let last_from = match sides.last().expect("sides") { Flat::Line(a, _) => *a, Flat::Arc { .. } => root };
            let dir = [face[0] - last_from[0], face[1] - last_from[1]];
            let rise = dir[1].abs().max(1e-300);
            let beyond = [face[0] + dir[0] / rise * margin, face[1] + dir[1].signum() * margin];
            let mut outline = vec![Flat::Line([across, root[1]], root)];
            outline.extend(sides.iter().copied());
            outline.push(Flat::Line(face, beyond));
            outline.push(Flat::Line(beyond, [across, beyond[1]]));
            outline.push(Flat::Line([across, beyond[1]], [across, root[1]]));
            cutters.push(outline);
        }
        match cutters.len() {
            1 => Ok(cutters.pop().expect("one cutter")),
            // A double groove: one cutter through both, joined along the
            // plane's hair past the land.
            2 => {
                let (lower, upper) = (cutters.pop().expect("lower"), cutters.pop().expect("upper"));
                let mut outline = Vec::new();
                // Upper from its root up, round to the hair, down the hair
                // to the lower groove's far end, then the lower groove back
                // to its root and across the land.
                let upper_root = start(&upper[1..]);
                let lower_root = start(&lower[1..]);
                outline.extend(upper[1..upper.len() - 2].iter().copied());
                let upper_beyond = upper[upper.len() - 3].end();
                let lower_beyond = lower[lower.len() - 3].end();
                outline.push(Flat::Line(upper_beyond, [across, upper_beyond[1]]));
                outline.push(Flat::Line([across, upper_beyond[1]], [across, lower_beyond[1]]));
                outline.push(Flat::Line([across, lower_beyond[1]], lower_beyond));
                outline.extend(lower[1..lower.len() - 2].iter().rev().map(Flat::reversed));
                if (upper_root[1] - lower_root[1]).abs() > self.tolerance {
                    outline.push(Flat::Line(lower_root, upper_root));
                }
                Ok(outline)
            }
            _ => Err("the face is not prepared".into()),
        }
    }
}

fn start(sides: &[Flat]) -> [f64; 2] {
    match sides.first().expect("a side") { Flat::Line(a, _) => *a, Flat::Arc { from, .. } => *from }
}

fn mirror_x(flat: &Flat, gap: f64) -> Flat {
    let m = |p: [f64; 2]| [gap - p[0], p[1]];
    match *flat {
        Flat::Line(a, b) => Flat::Line(m(a), m(b)),
        Flat::Arc { from, to, centre, through } => Flat::Arc { from: m(from), to: m(to), centre: m(centre), through: m(through) },
    }
}

fn mirror_y(flat: &Flat, t: f64) -> Flat {
    let m = |p: [f64; 2]| [p[0], t - p[1]];
    match *flat {
        Flat::Line(a, b) => Flat::Line(m(a), m(b)),
        Flat::Arc { from, to, centre, through } => Flat::Arc { from: m(from), to: m(to), centre: m(centre), through: m(through) },
    }
}

/// A circular arc from `a` to `b` (on one level) bulging `height` to the
/// side `height`'s sign says (+ = towards larger y).
fn arc_flat(a: [f64; 2], b: [f64; 2], height: f64) -> Flat {
    let half = (b[0] - a[0]).abs() * 0.5;
    let depth = height.abs();
    let radius = (half * half + depth * depth) / (2.0 * depth);
    let middle = [(a[0] + b[0]) * 0.5, a[1]];
    let centre = [middle[0], middle[1] - height.signum() * (radius - depth)];
    Flat::Arc { from: a, to: b, centre, through: [middle[0], middle[1] + height] }
}

/// The sides across one face of the joint from `from` to `to` (on level
/// `level`): straight, or a cap `height` high (out of the part, to `sign`)
/// running `overlap` onto each part past the groove's edges.
fn cap_sides(from: [f64; 2], to: [f64; 2], level: f64, height: f64, overlap: f64, sign: f64) -> Vec<Flat> {
    let mut sides = Vec::new();
    if height <= 0.0 {
        if (from[0] - to[0]).abs() > 1e-12 {
            sides.push(Flat::Line(from, to));
        }
        return sides;
    }
    // The cap's toes lie `overlap` beyond `from` and `to`.
    let outward = if to[0] < from[0] { 1.0 } else { -1.0 };
    let a = [from[0] + outward * overlap, level];
    let b = [to[0] - outward * overlap, level];
    if overlap > 0.0 { sides.push(Flat::Line(from, a)); }
    sides.push(arc_flat(a, b, height * sign));
    if overlap > 0.0 { sides.push(Flat::Line(b, to)); }
    sides
}

fn end(sides: &[Flat]) -> [f64; 2] {
    sides.last().expect("a side").end()
}

/// The strips a cap lays on each part's face past the groove's edges (`left`
/// and `right` are the groove's edges on that face).
fn toes(left: [f64; 2], right: [f64; 2], height: f64, overlap: f64) -> Vec<(usize, Flat)> {
    if height <= 0.0 || overlap <= 0.0 {
        return Vec::new();
    }
    let (low, high) = if left[0] <= right[0] { (left, right) } else { (right, left) };
    vec![
        (0, Flat::Line([low[0] - overlap, low[1]], low)),
        (1, Flat::Line(high, [high[0] + overlap, high[1]])),
    ]
}

/// The thin slab just inside a part under a side the bead lies against: the
/// side offset by the slab depth to whichever side `inside` says is the
/// part's (a line along its normal, an arc to the concentric arc). `None`
/// when neither side is inside the part.
fn slab(flat: &Flat, inside: &dyn Fn([f64; 2]) -> Result<bool, String>) -> Result<Option<Vec<Flat>>, String> {
    let depth = slab_depth();
    let offset = |sign: f64| -> Result<(Flat, [f64; 2]), String> {
        Ok(match *flat {
            Flat::Line(a, b) => {
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let length = (dx * dx + dy * dy).sqrt();
                if length <= 1e-300 { return Err("a contact side has no length".into()); }
                let n = [-dy / length * sign * depth, dx / length * sign * depth];
                // The inner edge is pulled in along the side at each end, so
                // the slab stays in a part whose corner there is acute (down
                // to about 7 degrees; a prepared face leans at most 80 degrees,
                // leaving 10 at a closed root); the contact itself is covered
                // end to end.
                let pull = if length > 20.0 * depth { 8.0 * depth / length } else { 0.0 };
                let along = [dx * pull, dy * pull];
                let moved = Flat::Line([a[0] + n[0] + along[0], a[1] + n[1] + along[1]], [b[0] + n[0] - along[0], b[1] + n[1] - along[1]]);
                let probe = [(a[0] + b[0]) * 0.5 + n[0] * 0.5, (a[1] + b[1]) * 0.5 + n[1] * 0.5];
                (moved, probe)
            }
            Flat::Arc { from, to, centre, through } => {
                let radius = ((from[0] - centre[0]).powi(2) + (from[1] - centre[1]).powi(2)).sqrt();
                let scale = (radius + sign * depth) / radius;
                let at = |p: [f64; 2]| [centre[0] + (p[0] - centre[0]) * scale, centre[1] + (p[1] - centre[1]) * scale];
                let half = (radius + sign * depth * 0.5) / radius;
                let probe = [centre[0] + (through[0] - centre[0]) * half, centre[1] + (through[1] - centre[1]) * half];
                (Flat::Arc { from: at(from), to: at(to), centre, through: at(through) }, probe)
            }
        })
    };
    for sign in [1.0, -1.0] {
        let (moved, probe) = offset(sign)?;
        if inside(probe)? {
            let (a, b) = (start(std::slice::from_ref(flat)), flat.end());
            let (a2, b2) = (start(std::slice::from_ref(&moved)), moved.end());
            return Ok(Some(vec![*flat, Flat::Line(b, b2), moved.reversed(), Flat::Line(a2, a)]));
        }
    }
    Ok(None)
}
