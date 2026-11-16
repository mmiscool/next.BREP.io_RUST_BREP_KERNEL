//! Groove (butt) welds: the bead fills the joint two butting parts' faces
//! make, with an optional cap on the face the groove opens to and a root bead
//! on the other. The joint preparation (bevel, J or U) can be cut into the
//! parts by the weld. In an assembly it is cut in context: the prepared body
//! replaces the instance in the assembly only, and the part document, the
//! component and its other instances keep the unprepared body.
use super::common;
use super::fit::{self, Seam, Side, Trim};

#[path = "weld_edge.rs"]
pub(crate) mod edge;
#[path = "weld_flare.rs"]
mod flare;
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
    /// Two round faces (bars or bends): the flare between them.
    FlareV,
    /// A round face against a flat one.
    FlareBevel,
}

const SHAPES: [(&str, Shape); 7] = [("SQUARE", Shape::Square), ("V", Shape::V), ("BEVEL", Shape::Bevel), ("U", Shape::U), ("J", Shape::J),
    ("FLARE_V", Shape::FlareV), ("FLARE_BEVEL", Shape::FlareBevel)];

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
    if shape.starts_with("FLARE") {
        hidden.extend(["grooveAngle", "rootFace", "rootRadius", "bevelFace", "rootReinforcement", "capOverlap", "prepare", "transition", "transitionSlope"]);
        return hidden;
    }
    hidden.push("flareDepth");
    if shape == "SQUARE" {
        // Prepare joint stays: it also lets a TAPER cut a thicker part.
        hidden.extend(["grooveAngle", "rootFace"]);
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
            "shape": {"type": "options", "label": "Groove", "options": ["SQUARE", "V", "BEVEL", "U", "J", "FLARE_V", "FLARE_BEVEL"], "default_value": "V",
                "hint": "SQUARE: both faces square, the root opening filled. V and U: both faces prepared. BEVEL and J: one face prepared (Bevelled face), the other square. FLARE_V: between two round faces (bars or bent edges). FLARE_BEVEL: between a round face and a flat one."},
            "double": {"type": "boolean", "label": "Double", "default_value": false,
                "hint": "Prepare and weld from both sides: the groove is mirrored about the middle of the thickness, a cap on each side."},
            "grooveAngle": {"type": "number", "label": "Groove angle", "default_value": 60,
                "hint": "The groove's included angle in degrees: split between the two faces for V and U, all on the bevelled face for BEVEL and J."},
            "rootFace": {"type": "number", "label": "Root face", "default_value": 0,
                "hint": "The unprepared land at the root, in model units (for a double groove, in the middle). The groove depth is the thickness less the root face."},
            "flareDepth": {"type": "number", "label": "Groove depth", "default_value": 0,
                "hint": "How deep a flare groove is filled from its top (the smaller round's tangent level), in model units. 0 = down to the root."},
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
            "transition": {"type": "options", "label": "Thickness transition", "options": ["TAPER", "WELD_SLOPE"], "default_value": "TAPER",
                "hint": "For parts of unequal thickness: TAPER cuts the thicker part down to the thinner at the slope below (Prepare joint on; in an assembly the cut is the assembly's, never the part's); WELD_SLOPE cuts no taper and builds the slope out of weld metal from the thicker part's edge down onto the thinner part (a step that must stay). Parts of equal thickness ignore it."},
            "transitionSlope": {"type": "number", "label": "Transition slope (1 in)", "default_value": 2.5,
                "hint": "The transition's run per unit of height difference: 2.5 is the 1:2.5 slope AWS D1.1 allows at most; ISO practice often uses 4."},
            "prepare": {"type": "boolean", "label": "Prepare joint", "default_value": true,
                "hint": "Cut the groove's bevel, J or U (and a TAPER into a thicker part) into the parts. In an assembly the cut belongs to the assembly: that instance shows it, while the part document and the part's other instances stay as they are. Off: the parts must already be prepared."},
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
    /// How far each part runs past the joint's thickness (the thinner
    /// part's) at the back face and at the face the groove opens to.
    steps: [[f64; 2]; 2],
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
            .ok_or_else(|| format!("unknown groove '{value}' (SQUARE, V, BEVEL, U, J, FLARE_V or FLARE_BEVEL)"))?,
        Some(_) => return Err("groove must be SQUARE, V, BEVEL, U, J, FLARE_V or FLARE_BEVEL".into()),
    };
    let double = boolean(ctx, "double", false, "Double")?;
    if matches!(shape, Shape::FlareV | Shape::FlareBevel) {
        return flare::build(ctx, flare::Settings {
            flare: if shape == Shape::FlareV { flare::Flare::V } else { flare::Flare::Bevel },
            double,
            cap: number(ctx, "reinforcement", 1.0, "reinforcement", false)?,
            generate: boolean(ctx, "generateSolid", true, "Generate weld solid")?,
            colour: match ctx.param("weldColor") {
                None => super::WELD_COLOUR.to_string(),
                Some(serde_json::Value::String(hex)) if super::is_hex_colour(hex) => hex.to_ascii_lowercase(),
                Some(_) => return Err("weld colour must be `#rrggbb`".into()),
            },
        });
    }
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
    let may_cut = boolean(ctx, "prepare", true, "Prepare joint")?;
    let prepare = shape != Shape::Square && may_cut;
    let taper = match ctx.param("transition") {
        None => true,
        Some(serde_json::Value::String(value)) if value == "TAPER" => true,
        Some(serde_json::Value::String(value)) if value == "WELD_SLOPE" => false,
        Some(_) => return Err("thickness transition must be TAPER or WELD_SLOPE".into()),
    };
    let slope = number(ctx, "transitionSlope", 2.5, "transition slope", true)?;
    let generate = boolean(ctx, "generateSolid", true, "Generate weld solid")?;
    let colour = match ctx.param("weldColor") {
        None => super::WELD_COLOUR.to_string(),
        Some(serde_json::Value::String(hex)) if super::is_hex_colour(hex) => hex.to_ascii_lowercase(),
        Some(_) => return Err("weld colour must be `#rrggbb`".into()),
    };
    let joint = joint(ctx)?;
    let t = joint.thickness;
    let tolerance = 1e-6_f64.max(joint.length.max(t) * 1e-12);
    let stepped = joint.steps.iter().flatten().any(|rise| *rise > 0.0);
    if stepped && taper {
        if !may_cut {
            return Err("the parts are of unequal thickness, and a TAPER transition cuts the thicker one: turn Prepare joint on, or choose the WELD_SLOPE transition".into());
        }
    }
    if matches!(shape, Shape::U | Shape::J) && root_face <= tolerance {
        return Err("a U or J groove needs a root face: its radius would run out along the back face to a feather edge".into());
    }
    if root_face >= t - tolerance && shape != Shape::Square {
        return Err(format!("the root face {root_face} leaves no groove in the {t} thick joint"));
    }
    // How each face is prepared (the first face is x ≤ 0, the second x ≥ gap).
    let prepared = |bevelled: bool, share: f64| -> Edge {
        match (shape, bevelled) {
            (Shape::Square | Shape::FlareV | Shape::FlareBevel, _) | (_, false) => Edge::Square,
            (Shape::V | Shape::Bevel, true) => Edge::Bevel(angle * share),
            (Shape::U | Shape::J, true) => Edge::J(radius, angle * share),
        }
    };
    let edges = match shape {
        Shape::Square | Shape::FlareV | Shape::FlareBevel => [Edge::Square, Edge::Square],
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
    // A component member is prepared in context: the cut body replaces the
    // member in this assembly only (the part document, the component and its
    // other instances keep the unprepared body; see `SceneMap::weld_prepared`).
    let member = |name: &str| ctx.scene.owning_component(name).is_some_and(|record| record.solids.iter().any(|solid| solid == name));
    if prepare || (stepped && taper) {
        // A cutter for each prepared face, past the faces it opens to and
        // past the seam's ends, reaching just across the face's plane (into
        // the root opening, never into the other part, which is not cut).
        let margin = t.max(joint.length) * 0.05 + 1.0;
        for (side, (name, body)) in parts.iter_mut().enumerate() {
            let mut tapers: Vec<(bool, f64)> = if taper {
                [(false, joint.steps[side][0]), (true, joint.steps[side][1])].into_iter().filter(|(_, rise)| *rise > 0.0).collect()
            } else {
                Vec::new()
            };
            if (!prepare || section.edges[side] == Edge::Square) && tapers.is_empty() { continue; }
            if let Some(other) = ctx.scene.context_member(name.as_str()).filter(|other| other.weld != ctx.id) {
                return Err(format!("{name} is already prepared in this assembly by {}: an instance takes one joint preparation in an assembly (prepare the part in its own document for more)", other.weld));
            }
            let shells = body.shells.len();
            // Along the seam the cut runs out past an end only where the part
            // ends there too; a part that runs on keeps its square face past
            // the joint.
            let [before, after] = joint.ends[side].map(|ends| if ends { margin } else { 0.0 });
            let cut_seam = Seam::line(joint.origin.sub(joint.d.scale(before)), joint.d, joint.length + before + after, None);
            let back = |x: f64, y: f64| at(x, y).sub(joint.d.scale(before));
            let mut outlines: Vec<(&str, Vec<Flat>)> = Vec::new();
            if prepare && section.edges[side] != Edge::Square {
                // Carried past the face by at least the part's step, so a
                // thicker part's flank runs on to its own face; where the
                // part is tapered there, the cut runs on along the taper
                // (a flank flatter than the taper would cut under it).
                let rise = joint.steps[side][0].max(joint.steps[side][1]);
                let away = if side == 0 { -1.0 } else { 1.0 };
                let mut run_on = [None; 2];
                tapers.retain(|(top, _)| {
                    // The groove opens to the top, and to the back too when double.
                    let grooved = *top || double;
                    if grooved {
                        run_on[usize::from(*top)] = Some([away * slope, if *top { 1.0 } else { -1.0 }]);
                    }
                    !grooved
                });
                outlines.push(("Face", section.cutter(side, margin + rise, run_on)?));
            }
            for (top, rise) in tapers {
                outlines.push(("Taper", section.taper(side, top, rise, slope, margin)?));
            }
            for (kind, flats) in outlines {
                let outline: Vec<Side> = flats.into_iter().map(|side| side.place(&back)).collect();
                let mut cutter = fit::sweep_outline(&cut_seam, &outline, &Trim::default())?;
                // The faces the cut leaves on the part are named for the weld
                // and the side, so the two prepared parts never share a name.
                for (i, face) in cutter.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
                    face.name = Some(format!("{}:Prep{}{kind}{}", ctx.id, side + 1, i + 1));
                }
                *body = common::subtract_solid(body.clone(), cutter)
                    .map_err(|refusal| format!("could not prepare {name}: {refusal}"))?;
            }
            // An instance prepared in context stays one body.
            if member(name) && body.shells.len() != shells {
                return Err(format!("preparing {name} would split it into {} bodies", body.shells.len()));
            }
            cut[side] = true;
        }
    }
    let mut beads: Vec<(String, crate::BrepSolid)> = Vec::new();
    if generate {
        // How each face of the joint is finished: a cap (or flush), and where
        // a thicker part runs past the joint there, its transition.
        let finish = |face: usize, height: f64, welded: bool| -> Finish {
            let step = (0..2).find(|side| joint.steps[*side][face] > 0.0).map(|side| Step {
                side,
                rise: joint.steps[side][face],
                // A taper is cut into the part; a weld slope is built only
                // on a face the weld finishes.
                slope: if taper || !welded { None } else { Some(slope) },
            });
            Finish { height, overlap, step }
        };
        let regions = section.regions(finish(1, cap, true), finish(0, if double { cap } else { root_bead }, double || root_bead > 0.0))?;
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
            result.added.push(if member(&name) {
                crate::feature_pipeline::component::register_context_member(body, &name)
            } else {
                common::register_added(body, &name)
            });
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
    // The joint's thickness is where both parts are; a thicker part runs
    // past it at one face or both (a step the weld's transition meets).
    let (aw0, aw1) = (pa0.max(pb0), pa1.min(pb1));
    // Read against the parts' thickness, not the faces' run along the seam.
    let across = 1e-9 * (pa1 - pa0).max(pb1 - pb0).max(1.0);
    if aw1 - aw0 <= across {
        return Err(format!("the parts of '{first}' and '{second}' do not overlap through their thickness across the joint"));
    }
    let step = |run: f64| if run > across * 1e3 { run } else { 0.0 };
    let steps = [[step(aw0 - pa0), step(pa1 - aw1)], [step(aw0 - pb0), step(pb1 - aw1)]];
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
        steps,
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

/// How the weld finishes one face of the joint.
#[derive(Clone, Copy)]
struct Finish {
    /// The cap's height (0: flush).
    height: f64,
    /// How far the cap runs onto each part past the groove's edge.
    overlap: f64,
    /// A part that runs past the joint's thickness at this face.
    step: Option<Step>,
}

/// A thicker part's step past the joint at one face.
#[derive(Clone, Copy)]
struct Step {
    side: usize,
    rise: f64,
    /// The weld slope's run per unit rise; `None` where the part is tapered
    /// (or the face is not welded), leaving no toe on the thicker part.
    slope: Option<f64>,
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
    /// `top` finishes the face the groove opens to, `back` the other.
    fn regions(&self, top: Finish, back: Finish) -> Result<Vec<Region>, String> {
        let t = self.thickness;
        let left = self.boundary(0)?;
        let right = self.boundary(1)?;
        let mut regions = Vec::new();
        if self.gap > self.tolerance {
            // One region: up the second face's boundary, across the top,
            // down the first's, across the bottom.
            let (l1, r1) = (end(&left), end(&right));
            let (l0, r0) = (start(&left), start(&right));
            let mut outline: Vec<Flat> = right.clone();
            let mut contacts: Vec<(usize, Flat)> = left.iter().map(|flat| (0, *flat)).chain(right.iter().map(|flat| (1, *flat))).collect();
            let (sides, laid) = self.finish([(r1, 1), (l1, 0)], 1.0, &top)?;
            outline.extend(sides);
            contacts.extend(laid);
            outline.extend(left.iter().rev().map(Flat::reversed));
            let (sides, laid) = self.finish([(l0, 0), (r0, 1)], -1.0, &back)?;
            outline.extend(sides);
            contacts.extend(laid);
            regions.push(Region { outline, contacts });
            return Ok(regions);
        }
        // No root opening: the faces meet at the root. Each groove opening to
        // a face is a region of its own (with that face's cap); a face with
        // no groove there has only its cap (or its transition).
        for (opening, finish, sign) in [(t, top, 1.0), (0.0, back, -1.0)] {
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
                // Square at this face (or the back of a single groove): a cap
                // (or a transition) alone.
                let point = [0.0, opening];
                let (outline, contacts) = self.finish([(point, 1), (point, 0)], sign, &finish)?;
                if !outline.is_empty() {
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
            let (sides, laid) = self.finish([(r1, 1), (l1, 0)], sign, &finish)?;
            outline.extend(sides);
            outline.extend(l.iter().rev().map(Flat::reversed));
            let mut contacts: Vec<(usize, Flat)> = l.iter().map(|flat| (0, *flat)).chain(r.iter().map(|flat| (1, *flat))).collect();
            contacts.extend(laid);
            regions.push(Region { outline, contacts });
        }
        Ok(regions)
    }

    /// The sides across one face of the joint from `ends[0]` to `ends[1]`
    /// (each a groove edge on that face, with its part's side), bulging to
    /// `sign`, and the strips of them that lie on the parts.
    fn finish(&self, ends: [([f64; 2], usize); 2], sign: f64, finish: &Finish) -> Result<(Vec<Flat>, Vec<(usize, Flat)>), String> {
        let [(from, from_side), (to, _)] = ends;
        let mut overlaps = [finish.overlap; 2];
        if let Some(step) = finish.step {
            match step.slope {
                // A taper is cut down to the groove's edge: no toe runs onto it.
                None => overlaps[step.side] = 0.0,
                Some(slope) => return self.weld_slope(ends, sign, finish.overlap, step, slope),
            }
        }
        let from_high = from[0] > to[0] || (from[0] == to[0] && from_side == 1);
        let sides = cap_sides(from, to, from_high, from[1], finish.height, overlaps, sign);
        Ok((sides, toes(from, to, finish.height, overlaps)))
    }

    /// A weld slope at one face: from the thicker part's edge up its face
    /// (its last flank carried on, a square face straight up) to its corner
    /// past the joint, then down at the slope onto the thinner part, at
    /// least the overlap past its groove edge.
    fn weld_slope(&self, ends: [([f64; 2], usize); 2], sign: f64, overlap: f64, step: Step, slope: f64) -> Result<(Vec<Flat>, Vec<(usize, Flat)>), String> {
        let [(from, from_side), (to, _)] = ends;
        let thick = step.side;
        let (edge, near) = if from_side == thick { (from, to) } else { (to, from) };
        let dir = self.flank_direction(thick, sign > 0.0)?;
        let corner = [edge[0] + dir[0] * step.rise / dir[1].abs(), edge[1] + sign * step.rise];
        let toward = if thick == 0 { 1.0 } else { -1.0 };
        let landing = corner[0] + toward * slope * step.rise;
        let reach = near[0] + toward * overlap;
        let toe = [if toward > 0.0 { landing.max(reach) } else { landing.min(reach) }, near[1]];
        let up = Flat::Line(edge, corner);
        let mut sides = vec![up, Flat::Line(corner, toe)];
        let mut contacts = vec![(thick, up)];
        if (toe[0] - near[0]).abs() > 1e-12 {
            sides.push(Flat::Line(toe, near));
            contacts.push((1 - thick, Flat::Line(toe, near)));
        }
        if from_side != thick {
            sides = sides.iter().rev().map(Flat::reversed).collect();
        }
        Ok((sides, contacts))
    }

    /// The direction face `side`'s part runs on past the face the groove
    /// opens to (`top`) or past the back: its last flank, outward.
    fn flank_direction(&self, side: usize, top: bool) -> Result<[f64; 2], String> {
        // A single groove leaves the back square: the part's face runs
        // straight on past it.
        if !top && !self.double {
            return Ok([0.0, -1.0]);
        }
        let boundary = self.boundary(side)?;
        let (a, b) = match (top, boundary.last(), boundary.first()) {
            (true, Some(Flat::Line(a, b)), _) => (*a, *b),
            (false, _, Some(Flat::Line(a, b))) => (*b, *a),
            _ => return Err("the groove's edge at a stepped face is not straight".into()),
        };
        let dir = [b[0] - a[0], b[1] - a[1]];
        if dir[1].abs() <= 1e-12 {
            return Err("the groove's edge runs along a stepped face".into());
        }
        Ok(dir)
    }

    /// The cutter that tapers face `side`'s part where it runs `rise` past
    /// the face the groove opens to (`top`) or past the back: everything
    /// past that face on the joint's side of the line from the groove's
    /// edge rising `1 : slope` away from the joint, carried `margin` past
    /// the part's face and a hair across the joint plane.
    fn taper(&self, side: usize, top: bool, rise: f64, slope: f64, margin: f64) -> Result<Vec<Flat>, String> {
        let boundary = self.boundary(side)?;
        let edge = if top { end(&boundary) } else { start(&boundary) };
        let away = if side == 0 { -1.0 } else { 1.0 };
        let out = if top { 1.0 } else { -1.0 };
        let plane = if side == 0 { 0.0 } else { self.gap };
        let across = plane - away * (self.gap * 0.5).min(slab_depth()).max(slab_depth() * 0.5);
        let high = edge[1] + out * (rise + margin);
        let far = [edge[0] + away * slope * (rise + margin), high];
        Ok(vec![
            Flat::Line(edge, far),
            Flat::Line(far, [across, high]),
            Flat::Line([across, high], [across, edge[1]]),
            Flat::Line([across, edge[1]], edge),
        ])
    }

    /// The cutter that prepares face `side`'s part: the groove on that side
    /// of the joint plane, carried `margin` past each face it opens to and a
    /// hair across the plane into the root opening (the other part is not cut).
    /// Past a face where `run_on[face]` (0 the back, 1 the face the groove
    /// opens to) gives a direction, the cut runs on that way (a taper's)
    /// instead of along the groove's last flank.
    fn cutter(&self, side: usize, margin: f64, run_on: [Option<[f64; 2]>; 2]) -> Result<Vec<Flat>, String> {
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
            let dir = run_on[usize::from(upper)].unwrap_or([face[0] - last_from[0], face[1] - last_from[1]]);
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
/// running onto each part past the groove's edges by that part's overlap
/// (`overlaps[0]` past the lower edge in x, the first part's, `[1]` past the
/// higher). `from_high` says `from` is the higher edge.
fn cap_sides(from: [f64; 2], to: [f64; 2], from_high: bool, level: f64, height: f64, overlaps: [f64; 2], sign: f64) -> Vec<Flat> {
    let mut sides = Vec::new();
    let straight = |sides: &mut Vec<Flat>| if (from[0] - to[0]).abs() > 1e-12 { sides.push(Flat::Line(from, to)); };
    if height <= 0.0 {
        straight(&mut sides);
        return sides;
    }
    let (low, high) = if from_high { (to, from) } else { (from, to) };
    let (toe_low, toe_high) = ([low[0] - overlaps[0], level], [high[0] + overlaps[1], level]);
    if toe_high[0] - toe_low[0] <= 1e-12 {
        straight(&mut sides);
        return sides;
    }
    let (first, last) = if from_high { (toe_high, toe_low) } else { (toe_low, toe_high) };
    if (first[0] - from[0]).abs() > 1e-12 { sides.push(Flat::Line(from, first)); }
    sides.push(arc_flat(first, last, height * sign));
    if (last[0] - to[0]).abs() > 1e-12 { sides.push(Flat::Line(last, to)); }
    sides
}

fn end(sides: &[Flat]) -> [f64; 2] {
    sides.last().expect("a side").end()
}

/// The strips a cap lays on each part's face past the groove's edges (`a`
/// and `b` are the groove's edges on that face), each part's overlap long.
fn toes(a: [f64; 2], b: [f64; 2], height: f64, overlaps: [f64; 2]) -> Vec<(usize, Flat)> {
    if height <= 0.0 {
        return Vec::new();
    }
    let (low, high) = if a[0] <= b[0] { (a, b) } else { (b, a) };
    let mut strips = Vec::new();
    if overlaps[0] > 0.0 { strips.push((0, Flat::Line([low[0] - overlaps[0], low[1]], low))); }
    if overlaps[1] > 0.0 { strips.push((1, Flat::Line(high, [high[0] + overlaps[1], high[1]]))); }
    strips
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
