//! Close the free ends of two perpendicular sibling flanges. Edits live in the
//! sheet tree, so replay, subsequent sheet features and unfolding retain them.
//! Bend sectors are preserved; reliefs cut the wall ends at the bend tangency.
use super::common;
use crate::feature_pipeline::sheet_metal::{self, Flat, Hole, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult, SelectionProbe};
use crate::{make_arc, make_line, Vec3};
use std::f64::consts::{PI, TAU};

const EPS: f64 = 1e-7;

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.all_sheet_metal && probe.faces == 2 && probe.edges == 0
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(format!("sheet-metal closed corner: {error}")),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let picks = common::reference_names(ctx.param("faces"));
    if picks.len() != 2 {
        return Err("select the broad faces of two adjacent flanges".into());
    }
    let mut handle = None;
    let mut ids = Vec::new();
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    for pick in &picks {
        let Some(face) = ctx.scene.resolve_face(pick) else {
            result.unresolved.push(pick.clone());
            continue;
        };
        if handle.is_some_and(|h| h != face.handle) {
            return Err("both flanges must belong to the same sheet-metal body".into());
        }
        handle = Some(face.handle);
        let name = pick.split('[').next().unwrap_or(pick);
        let id = name
            .strip_suffix(":A")
            .or_else(|| name.strip_suffix(":B"))
            .ok_or("select broad flange faces, not side faces or bend surfaces")?;
        ids.push(id.to_string());
    }
    if !result.unresolved.is_empty() {
        return Ok(result);
    }
    let handle = handle.ok_or("missing sheet-metal body")?;
    let mut tree = sheet_metal::get_tree(handle).ok_or("selection is not sheet metal")?;
    let body = ctx
        .scene
        .solids
        .iter()
        .find(|(_, h)| **h == handle)
        .map(|(name, _)| name.clone())
        .ok_or("body is no longer in the scene")?;
    let treatment = ctx
        .param("treatment")
        .and_then(|v| v.as_str())
        .unwrap_or("Closed");
    let options = Options {
        treatment: treatment.to_string(),
        reverse: match ctx
            .param("overlap")
            .and_then(|v| v.as_str())
            .unwrap_or("First over second")
        {
            "First over second" => false,
            "Second over first" => true,
            _ => return Err("unknown overlap option".into()),
        },
        gap: number(ctx, "gap", 0.1)?,
        size: if matches!(treatment, "Closed" | "V Cutout") {
            0.0
        } else {
            number(ctx, "cutoutSize", 2.0)?
        },
        depth: if matches!(treatment, "U Cutout" | "V Cutout") {
            number(ctx, "cutoutDepth", 2.0)?
        } else {
            0.0
        },
        v_angles: if treatment == "V Cutout" {
            [number(ctx, "vAngleFirst", 45.0)?, number(ctx, "vAngleSecond", 45.0)?]
        } else {
            [45.0; 2]
        },
    };
    apply(&mut tree, [&ids[0], &ids[1]], &options)?;
    result
        .added
        .push(sheet_metal::register_folded(tree, &body)?);
    result.removed.push(body);
    Ok(result)
}

fn number(ctx: &FeatureContext, name: &str, default: f64) -> Result<f64, String> {
    let value = if ctx.param(name).is_some() {
        ctx.number(name)?
    } else {
        default
    };
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{name} must be finite and nonnegative"));
    }
    Ok(value)
}

struct Options {
    treatment: String,
    reverse: bool,
    gap: f64,
    size: f64,
    depth: f64,
    v_angles: [f64; 2],
}

/// Locate sibling hinges that actually share an endpoint, including nested walls.
fn pair<'a>(flat: &'a Flat, ids: [&str; 2]) -> Option<(&'a Flat, usize, usize)> {
    let a = flat
        .edges
        .iter()
        .position(|e| e.bend.as_ref().is_some_and(|b| b.child.id == ids[0]));
    let b = flat
        .edges
        .iter()
        .position(|e| e.bend.as_ref().is_some_and(|b| b.child.id == ids[1]));
    if let (Some(a), Some(b)) = (a, b) {
        return Some((flat, a, b));
    }
    for edge in &flat.edges {
        if let Some(bend) = &edge.bend {
            if let Some(found) = pair(&bend.child, ids) {
                return Some(found);
            }
        }
    }
    None
}

fn apply(tree: &mut SheetTree, ids: [&str; 2], options: &Options) -> Result<(), String> {
    if ids[0] == ids[1] {
        return Err("select two different flanges".into());
    }
    if !matches!(
        options.treatment.as_str(),
        "Closed" | "Circular Cutout" | "U Cutout" | "V Cutout"
    ) {
        return Err("unknown corner treatment".into());
    }
    if !options.gap.is_finite() || options.gap < 0.0 {
        return Err("gap must be finite and nonnegative".into());
    }
    let (parent, a, b) =
        pair(&tree.root, ids).ok_or("flanges must share a parent flat and a corner")?;
    let n = parent.outline.len();
    let (p, q) = if (a + 1) % n == b {
        (a, b)
    } else if (b + 1) % n == a {
        (b, a)
    } else {
        return Err("the selected hinges do not share a corner".into());
    };
    let point = |i: usize| parent.outline[i % n];
    let d = |i: usize| {
        let (a, b) = (point(i), point(i + 1));
        [b[0] - a[0], b[1] - a[1]]
    };
    let (dp, dq) = (d(p), d(q));
    let (lp, lq) = (dp[0].hypot(dp[1]), dq[0].hypot(dq[1]));
    if lp <= EPS
        || lq <= EPS
        || (dp[0] * dq[0] + dp[1] * dq[1]).abs() > EPS * lp * lq
        || dp[0] * dq[1] - dp[1] * dq[0] <= 0.0
    {
        return Err("only convex perpendicular corners are supported".into());
    }
    let bp = parent.edges[p].bend.as_ref().unwrap();
    let bq = parent.edges[q].bend.as_ref().unwrap();
    if (bp.angle_deg.abs() - 90.0).abs() > EPS || (bp.angle_deg - bq.angle_deg).abs() > EPS {
        return Err("both bends must be 90 degrees in the same direction".into());
    }
    let through = if options.reverse { ids[1] } else { ids[0] };
    let edits = [
        (bp.child.id.clone(), false, lp, bq.inside_radius),
        (bq.child.id.clone(), true, lq, bp.inside_radius),
    ];
    let thickness = tree.thickness;
    for (id, start, span, radius) in edits {
        let flat = tree.root.find_by_id_mut(&id).unwrap();
        // Preserve earlier cuts/branches by refusing shapes we cannot safely extend.
        if flat.outline.len() != 4
            || flat.edges.len() != 4
            || !flat.holes.is_empty()
            || !flat.hole_bends.is_empty()
            || !flat.outline_curves.is_empty()
            || flat.edges.iter().any(|e| e.bend.is_some())
        {
            return Err("use plain rectangular flanges before adding cuts or further bends".into());
        }
        let leg = flat.outline[1][0];
        let expected = [[0.0, 0.0], [leg, 0.0], [leg, span], [0.0, span]];
        if leg <= EPS
            || flat
                .outline
                .iter()
                .zip(expected)
                .any(|(a, b)| (a[0] - b[0]).abs() > EPS || (a[1] - b[1]).abs() > EPS)
        {
            return Err("flanges must have untreated rectangular ends".into());
        }
        let extension = if id == through {
            radius + thickness
        } else {
            radius - options.gap
        };
        if extension < -EPS {
            return Err("gap exceeds the available corner extension (inside bend radius)".into());
        }
        let end = if start { -extension } else { span + extension };
        for i in if start { [0, 1] } else { [2, 3] } {
            flat.outline[i][1] = end;
        }
        if options.treatment != "Closed" {
            let depth = if options.treatment == "Circular Cutout" {
                options.size * 0.5
            } else {
                options.depth
            };
            let width = if options.treatment == "V Cutout" {
                // Angles belong to selection order, independently of which wall overlaps.
                let angle = options.v_angles[usize::from(id != ids[0])];
                if !angle.is_finite() || angle <= 0.0 || angle >= 90.0 {
                    return Err("V angles must be finite and strictly between 0 and 90 degrees".into());
                }
                depth * angle.to_radians().tan()
            } else {
                options.size * 0.5
            };
            if !depth.is_finite()
                || !width.is_finite()
                || depth <= EPS
                || width <= EPS
                || depth >= leg - EPS
                || width >= span - EPS
            {
                return Err(
                    "cutout dimensions must be positive and smaller than the flange leg and span"
                        .into(),
                );
            }
            // A cut centered on the wall's bend-tangent corner opens to both free
            // boundaries. Exact arcs remain exact through fold and flat export.
            let at = |x: f64, y: f64| Vec3::new(x, end + y, 0.0);
            let curves = match options.treatment.as_str() {
                "Circular Cutout" => vec![make_arc(
                    at(0.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    width,
                    0.0,
                    TAU,
                )?],
                "U Cutout" => {
                    if depth < width {
                        return Err("U cutout depth must be at least half its width".into());
                    }
                    vec![
                        make_line(at(-width, -width), at(depth - width, -width))?,
                        make_arc(
                            at(depth - width, 0.0),
                            Vec3::new(1.0, 0.0, 0.0),
                            Vec3::new(0.0, 1.0, 0.0),
                            width,
                            -PI / 2.0,
                            PI / 2.0,
                        )?,
                        make_line(at(depth - width, width), at(-width, width))?,
                        make_line(at(-width, width), at(-width, -width))?,
                    ]
                }
                _ => vec![
                    make_line(at(-depth, -2.0 * width), at(depth, 0.0))?,
                    make_line(at(depth, 0.0), at(-depth, 2.0 * width))?,
                    make_line(at(-depth, 2.0 * width), at(-depth, -2.0 * width))?,
                ],
            };
            flat.holes.push(Hole::through(curves));
        }
    }
    Ok(())
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type":"SM.CLOSED_CORNER", "shortName":"SM.CC", "longName":"SM Closed Corner",
        "ribbonPath":"Home/Sheet Metal/Closed Corner", "commandSize":"Compact", "displayBuilder":false,
        "inputParamsSchema": {
            "id":{"type":"string","default_value":null},
            "faces":{"type":"reference_selection","selectionFilter":["FACE"],"multiple":true,"default_value":null,
                "hint":"Select the broad faces of two adjacent rectangular flanges with 90° bends. Selection order controls overlap."},
            "treatment":{"label":"Corner treatment","type":"options","options":["Closed","Circular Cutout","U Cutout","V Cutout"],"default_value":"Closed"},
            "overlap":{"type":"options","options":["First over second","Second over first"],"default_value":"First over second","hint":"Choose which flange extends across the other's end."},
            "gap":{"type":"number","default_value":0.1,"min":0,"hint":"Clearance between the butting end and the overlapping flange; no greater than the inside bend radius."},
            "cutoutSize":{"label":"Cutout diameter / width","type":"number","default_value":2,"min":0,"hint":"Circular diameter, or U width at the bend tangent."},
            "cutoutDepth":{"label":"Cutout depth","type":"number","default_value":2,"min":0,"hint":"U/V reach along the flange from the bend tangent."},
            "vAngleFirst":{"label":"V angle on first flange","type":"number","default_value":45,"min":0,"max":90,"hint":"Angle in degrees between the first selected flange's V edge and the direction away from its bend; strictly between 0 and 90."},
            "vAngleSecond":{"label":"V angle on second flange","type":"number","default_value":45,"min":0,"max":90,"hint":"Independent V edge angle on the second selected flange; strictly between 0 and 90 degrees."}
        }
    })
}

/// The schema form displays only the dimensions used by the active treatment.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    match values.get("treatment").and_then(|v| v.as_str()).unwrap_or("Closed") {
        "Closed" => vec!["cutoutSize", "cutoutDepth", "vAngleFirst", "vAngleSecond"],
        "Circular Cutout" => vec!["cutoutDepth", "vAngleFirst", "vAngleSecond"],
        "U Cutout" => vec!["vAngleFirst", "vAngleSecond"],
        "V Cutout" => vec!["cutoutSize"],
        _ => vec![],
    }
}

