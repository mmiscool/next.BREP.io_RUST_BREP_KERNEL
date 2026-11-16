//! History-owned fillet welds. Seam geometry is published even without a bead.
//! Assembly references resolve in world space; the bead belongs to this document.
use super::common;
use crate::feature_pipeline::{FeatureContext, FeatureResult, Frame, ScenePoint, SelectionProbe};
use crate::Vec3;

#[path = "weld_fit.rs"]
mod fit;
#[path = "weld_groove.rs"]
pub(crate) mod groove;
#[path = "weld_lap.rs"]
pub(crate) mod lap;

/// How the bead section is placed around a straight seam.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Placement {
    /// Fit the joint faces; use the manual orientation where none fit.
    Auto,
    /// Fit the joint faces or fail by name.
    Fitted,
    /// Orientation only, as authored.
    Manual,
}

/// The Placement dropdown; AUTO when the key is absent (its schema default).
fn placement(ctx: &FeatureContext) -> Result<Placement, String> {
    match ctx.param("placement") {
        Some(serde_json::Value::String(value)) => match value.as_str() {
            "AUTO" => Ok(Placement::Auto),
            "FITTED" => Ok(Placement::Fitted),
            "MANUAL" => Ok(Placement::Manual),
            other => Err(format!("unknown placement '{other}' (AUTO, FITTED or MANUAL)")),
        },
        None => Ok(Placement::Auto),
        Some(_) => Err("placement must be AUTO, FITTED or MANUAL".into()),
    }
}

/// Which sides of the joint carry a bead.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Sides {
    /// The corner the seam picks.
    One,
    /// That corner and the mirror corner across the member standing on the
    /// joint (a T joint's far side).
    Both,
}

/// How the bead runs along the seam.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Intermittent {
    Continuous,
    /// Segments at a pitch, opposite each other on both sides.
    Chain,
    /// Segments at a pitch, the far side's half a pitch along.
    Staggered,
}

/// The bead's face between its toes.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Contour {
    Flat,
    Convex,
    Concave,
}

/// A dropdown's value, its schema default when the key is absent.
fn choice<T: Copy>(ctx: &FeatureContext, key: &str, label: &str, default: T, options: &[(&str, T)]) -> Result<T, String> {
    let names = || options.iter().map(|(name, _)| *name).collect::<Vec<_>>().join(", ");
    match ctx.param(key) {
        None => Ok(default),
        Some(serde_json::Value::String(value)) => options.iter().find(|(name, _)| *name == value.as_str()).map(|(_, v)| *v)
            .ok_or_else(|| format!("unknown {label} '{value}' ({})", names())),
        Some(_) => Err(format!("{label} must be one of {}", names())),
    }
}

/// The schema fields the fillet weld's dialog hides for these values.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    let mut hidden = Vec::new();
    if values.get("intermittent").and_then(serde_json::Value::as_str).unwrap_or("CONTINUOUS") == "CONTINUOUS" {
        hidden.extend(["segmentLength", "pitch"]);
    }
    if values.get("contour").and_then(serde_json::Value::as_str).unwrap_or("FLAT") == "FLAT" {
        hidden.push("contourHeight");
    }
    if values.get("placement").and_then(serde_json::Value::as_str) == Some("FITTED") {
        hidden.push("orientation");
    }
    hidden
}

/// The stretches of seam (from..to, by length from its start) an
/// intermittent weld lays: segments of `segment` every `pitch`, as many as
/// fit, the pattern centred on the seam; `half` moves them half a pitch
/// along (the far side of a staggered weld), keeping those that still fit.
fn stretches(total: f64, segment: f64, pitch: f64, half: bool) -> Result<Vec<(f64, f64)>, String> {
    let tolerance = 1e-9 * total.max(1.0);
    if segment > total + tolerance {
        return Ok(Vec::new());
    }
    let count = ((total - segment) / pitch + 1e-9).floor() + 1.0;
    if !(count <= MAX_SEGMENTS as f64) {
        return Err(format!("{count} intermittent segments on the {total} long seam; at most {MAX_SEGMENTS} (lengthen the pitch)"));
    }
    let count = count as i64;
    let margin = (total - (segment + (count - 1) as f64 * pitch)) * 0.5;
    let shift = if half { pitch * 0.5 } else { 0.0 };
    Ok((-1..=count).map(|i| margin + shift + i as f64 * pitch)
        .filter(|start| *start >= -tolerance && start + segment <= total + tolerance)
        .map(|start| (start.max(0.0), (start + segment).min(total)))
        .collect())
}

/// The most segments one intermittent side may lay: each is its own body.
const MAX_SEGMENTS: usize = 1000;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    build(ctx, None).unwrap_or_else(|error| ctx.fail(format!("weld: {error}")))
}

/// A tack weld: `count` short fillet segments of `length` each, the first
/// and last at the seam's ends (one tack sits in the middle).
#[derive(Clone, Copy, Debug)]
struct Tacks {
    length: f64,
    count: usize,
}

/// Tack welds (`WELD.TACK`): short fillet beads spread along a seam, fitted
/// and checked exactly as fillet welds are.
pub fn execute_tack(ctx: &FeatureContext) -> FeatureResult {
    let tacks = (|| -> Result<Tacks, String> {
        let length = common::optional_number(ctx, "tackLength")?.unwrap_or(5.0);
        if !length.is_finite() || length <= 0.0 {
            return Err("tack length must be finite and positive".into());
        }
        let count = common::optional_number(ctx, "count")?.unwrap_or(3.0);
        if !count.is_finite() || count < 1.0 || count.fract() != 0.0 || count > MAX_SEGMENTS as f64 {
            return Err(format!("tack count must be a whole number from 1 to {MAX_SEGMENTS}"));
        }
        Ok(Tacks { length, count: count as usize })
    })();
    match tacks {
        Ok(tacks) => build(ctx, Some(tacks)),
        Err(error) => Err(error),
    }
    .unwrap_or_else(|error| ctx.fail(format!("tack weld: {error}")))
}

/// The tack weld's schema: the fillet weld's seam, legs, sides, placement and
/// colour, with the tacks' length and count in place of its run options.
pub fn tack_schema() -> serde_json::Value {
    let mut schema = schema();
    schema["type"] = "WELD.TACK".into();
    schema["shortName"] = "TACK".into();
    schema["longName"] = "Tack weld".into();
    schema["ribbonPath"] = "Welding/Sheet metal and lap/Tack weld".into();
    schema["commandSize"] = "Compact".into();
    let fields = schema["inputParamsSchema"].as_object_mut().expect("fields");
    for key in ["intermittent", "segmentLength", "pitch", "allAround", "contour", "contourHeight"] {
        fields.remove(key);
    }
    fields.insert("tackLength".into(), serde_json::json!({"type": "number", "label": "Tack length", "default_value": 5,
        "hint": "Length of each tack along the seam, in model units."}));
    fields.insert("count".into(), serde_json::json!({"type": "number", "label": "Count", "default_value": 3,
        "hint": "How many tacks: the first and last at the seam's ends, the rest evenly between (one tack sits in the middle)."}));
    schema
}

/// The schema fields the tack weld's dialog hides for these values.
pub fn tack_hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    hidden_params(values).into_iter().filter(|key| *key == "orientation").collect()
}

fn build(ctx: &FeatureContext, tacks: Option<Tacks>) -> Result<FeatureResult, String> {
    let (seams, joint_faces) = selected_seam(ctx.scene, ctx.param("seam"))?;
    let all_around = match ctx.param("allAround") {
        None => false,
        Some(serde_json::Value::Bool(value)) => *value,
        Some(_) => return Err("All around must be a boolean".into()),
    };
    let seams = if all_around { around(ctx, seams, &joint_faces)? } else { seams };
    let seam = seams[0].clone();
    let curves: Vec<crate::NurbsCurve> = common::resolve_path_chain(ctx, &seams)?.into_iter().map(|segment| segment.curve).collect();
    let curve = curves.first().ok_or("the seam is empty")?;
    let [t0, t1] = curve.domain()?;
    let anchor = curve.evaluate((t0 + t1) * 0.5)?;
    let size = common::optional_number(ctx, "legSize")?.unwrap_or(3.0);
    if !size.is_finite() || size <= 0.0 {
        return Err("leg size must be finite and positive".into());
    }
    let second = common::optional_number(ctx, "legSize2")?.unwrap_or(0.0);
    if !second.is_finite() || second < 0.0 {
        return Err("second leg size must be finite and not negative (0 = equal legs)".into());
    }
    let second = if second == 0.0 { size } else { second };
    let swap = match ctx.param("swapLegs") {
        None => false,
        Some(serde_json::Value::Bool(value)) => *value,
        Some(_) => return Err("Swap legs must be a boolean".into()),
    };
    let legs = if swap { [second, size] } else { [size, second] };
    let placement = placement(ctx)?;
    let colour = match ctx.param("weldColor") {
        None => WELD_COLOUR.to_string(),
        Some(serde_json::Value::String(hex)) if is_hex_colour(hex) => hex.to_ascii_lowercase(),
        Some(_) => return Err("weld colour must be `#rrggbb`".into()),
    };
    let angle = common::optional_number(ctx, "orientation")?
        .unwrap_or(0.0)
        .to_radians();
    if !angle.is_finite() {
        return Err("orientation must be finite".into());
    }
    let sides = choice(ctx, "sides", "sides", Sides::One, &[("ONE", Sides::One), ("BOTH", Sides::Both)])?;
    let intermittent = choice(ctx, "intermittent", "intermittent", Intermittent::Continuous,
        &[("CONTINUOUS", Intermittent::Continuous), ("CHAIN", Intermittent::Chain), ("STAGGERED", Intermittent::Staggered)])?;
    let contour = choice(ctx, "contour", "contour", Contour::Flat,
        &[("FLAT", Contour::Flat), ("CONVEX", Contour::Convex), ("CONCAVE", Contour::Concave)])?;
    let contour = match contour {
        Contour::Flat => None,
        bulge => {
            let height = common::optional_number(ctx, "contourHeight")?.unwrap_or(0.5);
            if !height.is_finite() || height <= 0.0 {
                return Err("contour height must be finite and positive".into());
            }
            Some(if bulge == Contour::Convex { height } else { -height })
        }
    };
    let intermittent = if tacks.is_some() { Intermittent::Chain } else { intermittent };
    let pattern = match intermittent {
        _ if tacks.is_some() => None,
        Intermittent::Continuous => None,
        _ => {
            let segment = common::optional_number(ctx, "segmentLength")?.unwrap_or(10.0);
            let pitch = common::optional_number(ctx, "pitch")?.unwrap_or(25.0);
            if !segment.is_finite() || segment <= 0.0 {
                return Err("segment length must be finite and positive".into());
            }
            if !pitch.is_finite() || pitch <= segment {
                return Err("pitch (centre to centre) must be finite and longer than the segment length".into());
            }
            Some((segment, pitch))
        }
    };
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let generate = match ctx.param("generateSolid") {
        None => true,
        Some(serde_json::Value::Bool(value)) => *value,
        Some(_) => return Err("Generate weld solid must be a boolean".into()),
    };
    let owner = ctx.scene.resolve_edge(&seam).and_then(|edge| {
        ctx.scene.solids.iter().filter(|(_, handle)| **handle == edge.handle)
            .map(|(name, _)| name.clone()).min()
    });
    // The seam's runs: straight lines and circular arcs, head to tail.
    let mut runs = Vec::new();
    let mut shape: Option<String> = None;
    for (index, curve) in curves.iter().enumerate() {
        match fit::Seam::from_curve(curve, owner.clone())? {
            Some(run) => runs.push(run),
            None => {
                shape = Some(if curves.len() == 1 {
                    "the seam is neither straight nor a circular arc".into()
                } else {
                    format!("segment {} of the seam is neither straight nor a circular arc", index + 1)
                });
                break;
            }
        }
    }
    let chain = match shape {
        Some(reason) => Err(reason),
        None => fit::Chain::new(runs),
    };
    let chain = match chain {
        Ok(chain) => chain,
        Err(reason) => {
            faces_hold_seam_edge(ctx.scene, &seam, &joint_faces)?;
            if generate {
                not_built(&mut result, &seam, format!("has no weld bead: {reason}; the weld and its callout stand"));
            }
            return Ok(finish(ctx, result, seam, curves, anchor));
        }
    };
    let first = &chain.runs[0];
    let frame = seam_frame(ctx, &seam, first.start, first.tangent)?;
    result.frames.push((format!("{}:Seam:Frame", ctx.id), frame));
    // A picked face carries some run of the seam, stands a bridgeable root
    // gap off a single straight seam, or holds the seam edge.
    for name in &joint_faces {
        let mut carried = false;
        for run in &chain.runs {
            carried = carried || fit::carries(ctx.scene, run, name)?;
        }
        if !carried && chain.is_single_line() {
            carried = fit::bridges(ctx.scene, first, name, legs)?;
        }
        if !carried {
            faces_hold_seam_edge(ctx.scene, &seam, std::slice::from_ref(name))?;
        }
    }
    let reach = legs[0].max(legs[1]) + 100.0 * crate::tolerance::WELD_FLOOR;
    let fitting = placement != Placement::Manual && generate || !joint_faces.is_empty();
    let fitted = if fitting {
        match chain.trims(reach) {
            Ok(trims) => chain.fit(ctx.scene, &joint_faces, &ctx.id, legs)?.map(|fits| (fits, trims)),
            Err(reason) => Err(reason),
        }
    } else {
        Err("manual placement".into())
    };
    let mut unfitted: Option<String> = None;
    let fitted = match fitted {
        Ok(joint) if placement != Placement::Manual => Some(joint),
        Err(reason) if placement == Placement::Fitted && generate => {
            return Err(format!("fitted placement: {reason}"));
        }
        Err(reason) if placement == Placement::Auto => {
            unfitted = Some(reason);
            None
        }
        _ => None,
    };
    if generate {
        // The stretches a side's bead runs along its own seam (the far side's
        // is as long as the far run): the whole seam, or the intermittent
        // segments (the far side's half a pitch along when staggered).
        let spans = |total: f64, far: bool| -> Result<Vec<(f64, f64)>, String> {
            if let Some(Tacks { length, count }) = tacks {
                let tolerance = 1e-9 * total.max(1.0);
                if length * count as f64 > total + tolerance {
                    return Err(format!("{count} tacks of length {length} do not fit on the {total} long seam"));
                }
                if count == 1 {
                    let from = (total - length) * 0.5;
                    return Ok(vec![(from, from + length)]);
                }
                let pitch = (total - length) / (count - 1) as f64;
                return Ok((0..count).map(|i| {
                    let from = pitch * i as f64;
                    (from, (from + length).min(total))
                }).collect());
            }
            let Some((segment, pitch)) = pattern else { return Ok(vec![(0.0, total)]) };
            let spans = stretches(total, segment, pitch, far && intermittent == Intermittent::Staggered)?;
            if spans.is_empty() {
                return Err(format!("segment length {segment} does not fit on the {total} long seam"));
            }
            Ok(spans)
        };
        // The bodies of one side's bead: one per stretch (a continuous bead is
        // one body), each the stretch's pieces swept and united, named
        // `{id}` first and `{id}:Bead{n}` after it.
        let first_body = std::cell::Cell::new(0usize);
        let lay = |chain: &fit::Chain, sections: &[[Vec3; 3]], trims: &[fit::Trim], far: bool| -> Result<Vec<(String, Vec<fit::Piece>, crate::BrepSolid)>, String> {
            // A convex face can stand past the legs: the mitres are cut for
            // the section's own reach then.
            let widest = fit::section_reach(sections, contour)? + 100.0 * crate::tolerance::WELD_FLOOR;
            let wider;
            let trims = if widest > reach {
                wider = chain.trims(widest).map_err(|reason| format!("with its contour, {reason}"))?;
                &wider[..]
            } else {
                trims
            };
            if let Some(height) = contour.filter(|height| *height < 0.0) {
                for [_, a, b] in sections {
                    // A concave face may not reach the root: its depth stays
                    // below the section's height over the line between the toes.
                    let reach = a.cross(*b).length() / b.sub(*a).length();
                    if -height >= reach {
                        return Err(format!("contour height {} reaches the root of the {reach} deep bead section", -height));
                    }
                }
            }
            let mut bodies = Vec::new();
            for (from, to) in spans(chain.length(), far)? {
                let pieces = chain.pieces(sections, trims, from, to);
                let n = first_body.get() + bodies.len();
                let name = if n == 0 { ctx.id.clone() } else { format!("{}:Bead{}", ctx.id, n + 1) };
                let body = fit::build_bead(&name, &pieces, contour)?;
                bodies.push((name, pieces, body));
            }
            Ok(bodies)
        };
        // MANUAL builds at the authored Orientation: about the seam frame on a
        // single straight seam, else from the plane the seam lies in (0 lays
        // leg 1 across the seam in that plane, leg 2 along its normal). An AUTO
        // weld with no corner turns the bead in quarter turns from it until the
        // bead sits outside every body, so it is never hidden inside the metal.
        let plane = chain.plane();
        type Bodies = Vec<(String, Vec<fit::Piece>, crate::BrepSolid)>;
        let manual = |turn: f64| -> Result<Result<(Bodies, Vec3), String>, String> {
            let a = angle + turn;
            let sections: Vec<[Vec3; 3]> = if chain.is_single_line() {
                let u = frame.x_axis.scale(a.cos()).add(frame.y_axis.scale(a.sin()));
                vec![[Vec3::ZERO, u.scale(legs[0]), first.tangent.cross(u).scale(legs[1])]]
            } else {
                let Some(up) = plane else {
                    return Ok(Err("the seam does not lie in one plane, so Orientation cannot place a bead".into()));
                };
                chain.runs.iter().map(|run| {
                    let side = up.cross(run.tangent);
                    let u = side.scale(a.cos()).add(up.scale(a.sin()));
                    let v = up.scale(a.cos()).sub(side.scale(a.sin()));
                    [Vec3::ZERO, u.scale(legs[0]), v.scale(legs[1])]
                }).collect()
            };
            let trims = match chain.trims(reach) {
                Ok(trims) => trims,
                Err(reason) => return Ok(Err(reason)),
            };
            // The section triangle's centroid, halfway along the first stretch.
            let bodies = lay(&chain, &sections, &trims, false)?;
            let piece = &bodies[0].1[0];
            let [root, toe_u, toe_v] = piece.section;
            let centroid = piece.run.at(0.5).add(piece.run.carry(0.5, root.add(toe_u.add(toe_v).scale(1.0 / 3.0))));
            Ok(Ok((bodies, centroid)))
        };
        let auto_manual = |reason: &str, result: &mut FeatureResult| -> Result<Bodies, String> {
            // A turn whose bead cannot be built (a section across an arc's axis)
            // is skipped; the next turn may build.
            let mut first_built: Option<Bodies> = None;
            let mut failure: Option<String> = None;
            for quarter in 0..4 {
                let (bodies, centroid) = match manual(quarter as f64 * std::f64::consts::FRAC_PI_2) {
                    Ok(Ok(built)) => built,
                    Ok(Err(why)) => {
                        not_built(result, &seam, format!("has no weld bead: {reason}; {why}"));
                        return Ok(Vec::new());
                    }
                    Err(error) => {
                        failure.get_or_insert(error);
                        continue;
                    }
                };
                if outside_bodies(ctx.scene, centroid, &ctx.id)? {
                    not_built(result, &seam, format!("has an unfitted bead: {reason}; the bead is placed by Orientation{}",
                        if quarter == 0 { String::new() } else { format!(", turned {} degrees to sit outside the parts", quarter * 90) }));
                    return Ok(bodies);
                }
                first_built.get_or_insert(bodies);
            }
            match first_built {
                Some(bodies) => {
                    not_built(result, &seam, format!("has an unfitted bead: {reason}; no Orientation keeps the bead outside the parts"));
                    Ok(bodies)
                }
                None => {
                    not_built(result, &seam, format!("has no weld bead: {reason}; no Orientation builds one ({})", failure.unwrap_or_default()));
                    Ok(Vec::new())
                }
            }
        };
        // The far side of a single-run seam: its mirror joint, its chain and cuts.
        let far_side = |chain: &fit::Chain, near: &fit::Fit| -> Result<Result<(fit::Chain, fit::Fit, Vec<fit::Trim>), String>, String> {
            if chain.runs.len() != 1 {
                return Ok(Err("both sides is built on a single straight or circular seam; weld the far side's seam as its own weld".into()));
            }
            let (far_seam, far_fit) = match fit::other_side(ctx.scene, &chain.runs[0], near, &ctx.id, legs)? {
                Ok(found) => found,
                Err(reason) => return Ok(Err(reason)),
            };
            let far_chain = match fit::Chain::new(vec![far_seam]) {
                Ok(chain) => chain,
                Err(reason) => return Ok(Err(reason)),
            };
            Ok(far_chain.trims(reach).map(|trims| (far_chain, far_fit, trims)))
        };
        let bodies: Bodies = match &fitted {
            Some((fits, trims)) => {
                let sections: Vec<[Vec3; 3]> = fits.iter().map(|joint| [joint.root, joint.legs[0].scale(legs[0]), joint.legs[1].scale(legs[1])]).collect();
                let mut built = lay(&chain, &sections, trims, false)?;
                let mut checked = Ok(());
                for (_, pieces, body) in &built {
                    checked = chain.verify_pieces(ctx.scene, fits, legs, body, pieces, &ctx.id)?;
                    if checked.is_err() { break; }
                }
                match checked {
                    Ok(()) => {
                        let far_fit;
                        let mut joints: Vec<&fit::Fit> = fits.iter().collect();
                        // The far side: the mirror joint across the standing
                        // member, built and checked the same way. FITTED
                        // refuses without it; AUTO keeps the near side and says so.
                        let far = if sides == Sides::Both { Some(far_side(&chain, &fits[0])?) } else { None };
                        match far {
                            None => {}
                            Some(Ok((far_chain, fit, far_trims))) => {
                                let far_sections = [[fit.root, fit.legs[0].scale(legs[0]), fit.legs[1].scale(legs[1])]];
                                first_body.set(built.len());
                                let far = lay(&far_chain, &far_sections, &far_trims, true)?;
                                let mut outcome = Ok(());
                                for (_, pieces, body) in &far {
                                    outcome = far_chain.verify_pieces(ctx.scene, std::slice::from_ref(&fit), legs, body, pieces, &ctx.id)?;
                                    if outcome.is_err() { break; }
                                }
                                match outcome {
                                    Ok(()) => {
                                        far_fit = fit;
                                        joints.push(&far_fit);
                                        built.extend(far);
                                    }
                                    Err(reason) if placement == Placement::Fitted => return Err(format!("fitted placement: far side: {reason}")),
                                    Err(reason) => not_built(&mut result, &seam, format!("has a bead on one side only: far side: {reason}")),
                                }
                            }
                            Some(Err(reason)) if placement == Placement::Fitted => return Err(format!("fitted placement: both sides: {reason}")),
                            Some(Err(reason)) => not_built(&mut result, &seam, format!("has a bead on one side only: {reason}")),
                        }
                        // Leg order, each body once.
                        for solid in joints.iter().flat_map(|joint| joint.faces.iter().map(|face| face.solid.clone())) {
                            if !result.weld_joint_solids.contains(&solid) {
                                result.weld_joint_solids.push(solid);
                            }
                        }
                        built
                    }
                    Err(reason) if placement == Placement::Fitted => return Err(format!("fitted placement: {reason}")),
                    Err(reason) => auto_manual(&reason, &mut result)?,
                }
            }
            None => match &unfitted {
                Some(reason) => auto_manual(reason, &mut result)?,
                None => match manual(0.0)? {
                    Ok((bodies, _)) => bodies,
                    Err(why) => {
                        not_built(&mut result, &seam, format!("has no weld bead: {why}"));
                        Vec::new()
                    }
                },
            },
        };
        if sides == Sides::Both && fitted.is_none() && !bodies.is_empty() && result.fulfilment.is_none() {
            not_built(&mut result, &seam, "has a bead on one side only: both sides mirrors a fitted joint, and Placement is MANUAL".into());
        }
        for (name, _, solid) in bodies {
            result.added.push(common::register_added(solid, &name));
            paint_bead(&name, &colour);
        }
    }
    Ok(finish(ctx, result, seam, curves, anchor))
}

/// All around: one picked seam edge grown to the closed loop it runs on, the
/// picked edge first. Of the loops through the edge (one per face of its body
/// next to it), the one whose every edge is a weldable concave corner — a
/// part's foot outline on the face it stands on, a boss's outline on its
/// plate — is the seam; the other (the side face's own outline) has convex
/// edges. A seam that already closes is kept as picked.
fn around(ctx: &FeatureContext, seams: Vec<String>, joint_faces: &[String]) -> Result<Vec<String>, String> {
    let closes = |names: &[String]| -> Result<bool, String> {
        let curves = common::resolve_path_chain(ctx, names)?;
        let (Some(first), Some(last)) = (curves.first(), curves.last()) else { return Ok(false) };
        let [s0, _] = first.curve.domain()?;
        let [_, s1] = last.curve.domain()?;
        let (start, end) = (first.curve.evaluate(s0)?, last.curve.evaluate(s1)?);
        Ok(start.sub(end).length() <= 1e-6_f64.max(start.length() * 1e-12) * 10.0)
    };
    if closes(&seams)? {
        return Ok(seams);
    }
    let [picked] = seams.as_slice() else {
        return Err("all around grows one seam edge to its closed loop; select one edge, or every edge of the closed seam".into());
    };
    let edge = ctx.scene.resolve_edge(picked).ok_or("all around needs a resident seam edge (an edge of a part)")?;
    let loops: Vec<Vec<u64>> = crate::with_registered_solid_str(edge.handle, |solid| {
        let mut loops: Vec<Vec<u64>> = Vec::new();
        for record in solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops) {
            let mut ids: Vec<u64> = record.coedges.iter().map(|coedge| coedge.edge_id).collect();
            if !ids.contains(&edge.edge_id) { continue; }
            ids.sort_unstable();
            ids.dedup();
            if !loops.contains(&ids) { loops.push(ids); }
        }
        Ok(loops)
    })?;
    let name_of = |edge_id: u64| ctx.scene.edges.iter()
        .filter(|(_, other)| other.handle == edge.handle && other.edge_id == edge_id)
        .map(|(name, _)| name.clone()).min();
    let mut weldable: Vec<Vec<String>> = Vec::new();
    for ids in loops {
        let Some(mut names) = ids.iter().filter(|id| **id != edge.edge_id).map(|id| name_of(*id)).collect::<Option<Vec<String>>>() else { continue };
        names.insert(0, picked.clone());
        let Ok(segments) = common::resolve_path_chain(ctx, &names) else { continue };
        let mut runs = Vec::new();
        for segment in &segments {
            match fit::Seam::from_curve(&segment.curve, None)? {
                Some(run) => runs.push(run),
                None => break,
            }
        }
        if runs.len() != segments.len() { continue; }
        let Ok(chain) = fit::Chain::new(runs) else { continue };
        if !chain.is_closed() { continue; }
        if let Ok(Ok(_)) = chain.fit(ctx.scene, joint_faces, &ctx.id, [1e-3, 1e-3]) {
            weldable.push(names);
        }
    }
    match weldable.len() {
        1 => Ok(weldable.pop().expect("one loop")),
        0 => Err("all around: no closed loop through the seam edge is a concave corner all the way round; select every seam edge".into()),
        _ => Err("all around: several closed loops through the seam edge can be welded; select every seam edge".into()),
    }
}

/// Publish the seam path, its source name and the callout point.
fn finish(ctx: &FeatureContext, mut result: FeatureResult, seam: String, curves: Vec<crate::NurbsCurve>, anchor: Vec3) -> FeatureResult {
    result.weld_source_seam = Some(seam);
    result.paths.push((format!("{}:Seam", ctx.id), curves));
    result
        .points
        .push((format!("{}:Callout", ctx.id), ScenePoint::model(anchor)));
    result
}

/// The bead colour a new weld shows: a warm bronze that stands out against the
/// default face colour, so welds are easy to find in the model.
pub const WELD_COLOUR: &str = "#d9822b";

fn is_hex_colour(text: &str) -> bool {
    text.len() == 7 && text.starts_with('#') && text[1..].bytes().all(|b| b.is_ascii_hexdigit())
}

/// Give the bead solid the weld colour as its metadata `color`, the colour the
/// 3D view, the Metadata tab and the exporters read. `weldColor` records what
/// the weld last applied: a colour the user set on the bead in the Metadata
/// tab differs from it and is kept.
fn paint_bead(name: &str, colour: &str) {
    let record = crate::feature_pipeline::scene_metadata::own_record(name).unwrap_or_default();
    let current = record.get("color").and_then(serde_json::Value::as_str);
    let applied = record.get("weldColor").and_then(serde_json::Value::as_str);
    if current.is_none() || current == applied {
        let mut patch = serde_json::Map::new();
        patch.insert("color".into(), serde_json::Value::from(colour));
        patch.insert("weldColor".into(), serde_json::Value::from(colour));
        crate::feature_pipeline::scene_metadata::merge_record(name, &patch, true);
    }
}

/// Report what the weld could not build on its seam, beside the feature in the
/// history (the weld itself stands).
fn not_built(result: &mut FeatureResult, seam: &str, reason: String) {
    result.fulfilment = Some(crate::feature_pipeline::Fulfilment {
        requested: vec![seam.to_string()],
        applied: vec![seam.to_string()],
        rejected: vec![crate::feature_pipeline::Rejected {
            name: seam.to_string(),
            kind: crate::feature_pipeline::RejectedKind::NotBuilt,
            reason,
        }],
        ..Default::default()
    });
}

/// Whether the bead's centroid lies outside every resident body (beads of
/// other welds included).
fn outside_bodies(scene: &crate::feature_pipeline::SceneMap, centroid: Vec3, exclude: &str) -> Result<bool, String> {
    for (name, handle) in &scene.solids {
        if fit::owned_by(name, exclude) { continue; }
        let inside = crate::with_registered_solid_str(*handle, |body| {
            Ok(crate::classify_point(centroid, body, 1e-7)?.class != crate::PointClass::Out)
        })?;
        if inside { return Ok(false); }
    }
    Ok(true)
}

/// Resolve the explicit seam independently of selection order: one or more
/// seam edges or sketch paths that join head to tail (the first picked names
/// the seam). Faces are the joint faces (in selection order); parts and
/// components constrain the seam owner and must contain every seam edge.
fn selected_seam(scene: &crate::feature_pipeline::SceneMap, value: Option<&serde_json::Value>) -> Result<(Vec<String>, Vec<String>), String> {
    let names = common::unique_reference_names(value);
    let seams: Vec<String> = names.iter().filter(|name| scene.resolve_edge(name).is_some()
        || scene.resolve_path(name).is_some()
        || name.rsplit_once(':').is_some_and(|(owner, geometry)| geometry.len() > 1 && geometry.starts_with('G') && scene.resolve_path(owner).is_some()))
        .cloned().collect();
    if seams.is_empty() {
        // Two joint faces alone: the seam is the straight or circular edge they meet along.
        if names.len() == 2 && names.iter().all(|name| !scene.resolve_face_group(name).is_empty()) {
            let seam = fit::seam_between(scene, [&names[0], &names[1]])??;
            return Ok((vec![seam], names.clone()));
        }
        return Err("select the seam edges or sketch, or the two joint faces, alongside the part or component references".into());
    }
    let mut faces = Vec::new();
    for name in names.iter().filter(|name| !seams.contains(name)) {
        if !scene.resolve_face_group(name).is_empty() {
            faces.push(name.clone());
            continue;
        }
        for seam in &seams {
            let edge = scene.resolve_edge(seam).ok_or("part and component scope references require resident seam edges")?;
            let belongs = if let Some(handle) = scene.resolve_solid(name) {
                handle == edge.handle
            } else if let Some(component) = scene.resolve_component(name) {
                component.solids.iter().any(|solid| scene.resolve_solid(solid) == Some(edge.handle))
            } else {
                return Err(format!("reference '{name}' is not a resident face, part or component"));
            };
            if !belongs { return Err(format!("seam '{seam}' does not belong to selected reference '{name}'")); }
        }
    }
    Ok((seams, faces))
}

/// The v1 rule for a face picked beside a seam that cannot be fitted (curved
/// or multi-segment): the face must hold the seam edge.
fn faces_hold_seam_edge(scene: &crate::feature_pipeline::SceneMap, seam: &str, faces: &[String]) -> Result<(), String> {
    for name in faces {
        let edge = scene.resolve_edge(seam).ok_or("face references on a curved or multi-segment seam require a resident seam edge")?;
        let group = scene.resolve_face_group(name);
        let belongs = crate::with_registered_solid_str(edge.handle, |solid| {
            Ok(solid.shells.iter().flat_map(|shell| &shell.faces)
                .filter(|f| group.iter().any(|face| face.handle == edge.handle && face.face_id == f.id))
                .flat_map(|f| &f.loops).flat_map(|l| &l.coedges).any(|c| c.edge_id == edge.edge_id))
        })?;
        if !belongs { return Err(format!("seam '{seam}' does not belong to selected reference '{name}'")); }
    }
    Ok(())
}

fn direction(pose: crate::AffineTransform, vector: Vec3) -> Vec3 {
    let m = pose.elements;
    Vec3::new(m[0]*vector.x + m[1]*vector.y + m[2]*vector.z,
        m[4]*vector.x + m[5]*vector.y + m[6]*vector.z,
        m[8]*vector.x + m[9]*vector.y + m[10]*vector.z)
}

fn posed_frame(frame: Frame, pose: crate::AffineTransform) -> Frame {
    Frame { origin: pose.point(frame.origin), x_axis: direction(pose, frame.x_axis),
        y_axis: direction(pose, frame.y_axis), z_axis: direction(pose, frame.z_axis) }
}

/// Keep the manual orientation in the seam owner's coordinates, so an authored
/// rotation and an in-place solver rotation produce the same bead on rebuild.
fn seam_frame(ctx: &FeatureContext, seam: &str, start: Vec3, tangent: Vec3) -> Result<Frame, String> {
    if let Some(frame) = ctx.scene.frames.get(&format!("{seam}:Frame")) {
        return Ok(Frame { origin: start, ..*frame });
    }
    let mut owner: Option<(usize, crate::AffineTransform)> = None;
    for (id, record) in ctx.scene.components.iter().chain(ctx.scene.occurrences.iter()) {
        let mut consider = |name: &str, pose| {
            if seam.starts_with(&format!("{name}:")) && owner.as_ref().is_none_or(|(len, _)| name.len() > *len) {
                owner = Some((name.len(), pose));
            }
        };
        consider(id, record.transform);
        for (path, local) in &record.nested_poses {
            consider(&format!("{id}:{path}"), crate::feature_pipeline::component::compose(&record.transform, local)?);
        }
    }
    if let Some((_, pose)) = owner {
        let inverse = pose.rigid_inverse()?;
        let local = Frame::from_origin_normal(inverse.point(start), direction(inverse, tangent))?;
        Ok(posed_frame(local, pose))
    } else {
        Frame::from_origin_normal(start, tangent)
    }
}

/// Apply the seam owner's solved rigid motion without creating or discarding handles.
/// Compute every fallible transformation before committing the replacement bodies.
pub(crate) fn pose(result: &mut FeatureResult, delta: crate::AffineTransform) -> Result<(), String> {
    let bodies = result.added.iter().map(|added| {
        crate::with_registered_solid_str(added.handle, |solid| crate::transform_brep(solid, delta, false))
            .map(|body| (added.handle, body))
    }).collect::<Result<Vec<_>, _>>()?;
    let paths = result.paths.iter().map(|(name, curves)| {
        curves.iter().map(|curve| crate::transform_topology::transform_curve(curve, delta))
            .collect::<Result<Vec<_>, _>>().map(|curves| (name.clone(), curves))
    }).collect::<Result<Vec<_>, _>>()?;
    for (handle, body) in bodies { crate::replace_registered_solid(handle, body)?; }
    result.paths = paths;
    for (_, frame) in &mut result.frames { *frame = posed_frame(*frame, delta); }
    for (_, point) in &mut result.points { point.position = delta.point(point.position); }
    Ok(())
}

/// Whether `type_code` is a weld feature: the fillet weld `WELD`, or one of
/// the other weld kinds `WELD.<kind>` (groove, spot, plug, ...). Everything
/// that treats welds alike (history refits, post-solve posing, callouts,
/// component scoping) asks this, never the literal `WELD`.
pub fn is_weld_type(type_code: &str) -> bool {
    type_code == "WELD" || type_code.starts_with("WELD.")
}

/// Whether a weld descriptor may fit its bead to faces it does not name: the
/// history cache must then treat every resident body as an input.
pub(crate) fn fits_joint_faces(descriptor: &crate::feature_pipeline::FeatureDescriptor) -> bool {
    is_weld_type(&descriptor.feature_type)
        && descriptor.input_params.get("placement").and_then(|value| value.as_str()) != Some("MANUAL")
        && descriptor.input_params.get("generateSolid") != Some(&serde_json::Value::Bool(false))
}

/// The weld kinds the context bar does not offer (tack, plug or slot, edge,
/// corner, surfacing): their picks are too general to tell from a selection
/// (any edge, any face), so they are created from the Welding tab.
pub fn ribbon_only(_: &SelectionProbe) -> bool {
    false
}

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.edges + probe.sketches >= 1
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD", "shortName": "WELD", "longName": "Fillet weld",
        "ribbonPath": "Welding/Fillet and groove/Fillet weld", "commandSize": "Large", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE", "SKETCH", "FACE", "SOLID", "COMPONENT"], "multiple": true, "default_value": null,
                "hint": "Select the seam: one or more edges or sketch segments joined end to end. Add the joint faces to choose the corner (the first takes Leg size); parts and components constrain the seam owner and must contain the edges."},
            "legSize": {"type": "number", "label": "Leg size", "default_value": 3, "hint": "Fillet weld leg along the first joint face, in model units."},
            "legSize2": {"type": "number", "label": "Second leg", "default_value": 0, "hint": "Leg along the second joint face. 0 = equal legs."},
            "swapLegs": {"type": "boolean", "label": "Swap legs", "default_value": false, "hint": "Exchange which joint face each leg runs along."},
            "sides": {"type": "options", "label": "Sides", "options": ["ONE", "BOTH"], "default_value": "ONE",
                "hint": "ONE welds the corner the seam picks. BOTH also welds the mirror corner across the member standing on the joint (a T joint's far side), fitted and checked the same way."},
            "intermittent": {"type": "options", "label": "Intermittent", "options": ["CONTINUOUS", "CHAIN", "STAGGERED"], "default_value": "CONTINUOUS",
                "hint": "CONTINUOUS runs the bead along the whole seam. CHAIN lays segments at a pitch, centred on the seam (opposite each other on both sides). STAGGERED moves the far side's segments half a pitch along."},
            "segmentLength": {"type": "number", "label": "Segment length", "default_value": 10, "hint": "Length of each intermittent segment, in model units."},
            "pitch": {"type": "number", "label": "Pitch", "default_value": 25, "hint": "Centre-to-centre distance of intermittent segments; longer than the segment."},
            "allAround": {"type": "boolean", "label": "All around", "default_value": false,
                "hint": "Weld all the way round: one picked seam edge grows to the closed loop of concave corners it runs on (a part's foot on the plate it stands on, a boss's outline). Linked callouts show the all-around circle."},
            "contour": {"type": "options", "label": "Contour", "options": ["FLAT", "CONVEX", "CONCAVE"], "default_value": "FLAT",
                "hint": "The bead face between the toes: flat, or a circular arc bulging out of (CONVEX) or into (CONCAVE) the bead by Contour height."},
            "contourHeight": {"type": "number", "label": "Contour height", "default_value": 0.5, "hint": "How far a convex or concave face stands off the flat face at its middle, in model units."},
            "placement": {"type": "options", "label": "Placement", "options": ["AUTO", "FITTED", "MANUAL"], "default_value": "AUTO",
                "hint": "AUTO fits the bead into the corner of the two faces that meet along the seam and uses Orientation where no corner is found. FITTED requires the corner. MANUAL uses Orientation only."},
            "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
                "hint": "On by default: build a separate triangular bead along the seam's straight and circular segments, fitted to the joint faces and mitred at corners. Other seams remain annotation-only; turning off retains the weld and PMI."},
            "orientation": {"type": "number", "label": "Orientation", "default_value": 0, "hint": "Manual placement: rotate the bead around the seam in degrees (on a curved or cornered seam, from across the seam in the seam's plane)."},
            "weldColor": {"type": "color", "label": "Weld colour", "default_value": "#d9822b",
                "hint": "Colour of the bead solid in the 3D view and exports. A colour set on the bead in the Metadata tab is kept."},
            "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
        }
    })
}

