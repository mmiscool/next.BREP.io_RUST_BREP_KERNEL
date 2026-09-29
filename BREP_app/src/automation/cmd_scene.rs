//! Scene queries and selection: what exists, what is under a point, what is
//! selected, mass properties. Points are egui surface points; the viewport
//! offset is applied here.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PointArgs {
    pub x: f32,
    pub y: f32,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SelectArgs {
    /// `solid` | `face` | `edge` | `datum`
    pub kind: String,
    pub name: String,
}

/// The whole selection at once — the shape `selection` returns, minus the
/// position-keyed vertices.
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SelectionSetArgs {
    #[serde(default)]
    pub solids: Vec<String>,
    #[serde(default)]
    pub faces: Vec<String>,
    #[serde(default)]
    pub edges: Vec<String>,
    #[serde(default)]
    pub datums: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct VisibleArgs {
    pub name: String,
    pub visible: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MassArgs {
    /// A solid name; omitted = every resident solid.
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default = "one")]
    pub density: f64,
}
fn one() -> f64 {
    1.0
}

/// Surface point → viewport-local point (what the engine's pickers take).
fn local(ctx: &Ctx<'_>, x: f32, y: f32) -> Result<(f64, f64), String> {
    let r = ctx.app.viewport.last_rect().ok_or("the 3D viewport was not drawn last frame")?;
    Ok(((x - r.min.x) as f64, (y - r.min.y) as f64))
}

fn scene_entities(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "solids": parse(&engine.scene_entities_json()),
        "listing": parse(&engine.scene_listing_json()),
    })))
}

fn pick(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PointArgs = parse_args(args)?;
    let (lx, ly) = local(ctx, a.x, a.y)?;
    Ok(Outcome::Done(json!({ "candidates": parse(&ctx.app.docs.engine().pick_json(lx, ly)) })))
}

fn hover(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PointArgs = parse_args(args)?;
    let (lx, ly) = local(ctx, a.x, a.y)?;
    let changed = ctx.app.docs.engine_mut().hover_at(lx, ly);
    Ok(Outcome::Done(json!({ "changed": changed, "candidate": parse(&ctx.app.docs.engine().hover_json(lx, ly)) })))
}

fn select(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SelectArgs = parse_args(args)?;
    let ok = ctx.app.docs.engine_mut().select_by_name(&a.kind, &a.name);
    if !ok {
        return Err(format!("nothing selected: unknown kind `{}` or name `{}`", a.kind, a.name));
    }
    Ok(Outcome::Done(json!({ "selection": parse(&ctx.app.docs.engine().selection_json()) })))
}

fn selection_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SelectionSetArgs = parse_args(args)?;
    ctx.app
        .docs
        .engine_mut()
        .set_selection(&a.solids, &a.faces, &a.edges, &a.datums);
    Ok(Outcome::Done(json!({ "selection": parse(&ctx.app.docs.engine().selection_json()) })))
}

fn select_clear(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let changed = ctx.app.docs.engine_mut().clear_selection();
    Ok(Outcome::Done(json!({ "changed": changed })))
}

fn selection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "selection": parse(&ctx.app.docs.engine().selection_json()) })))
}

fn set_visible(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VisibleArgs = parse_args(args)?;
    let ok = ctx.app.docs.engine_mut().set_visible(&a.name, a.visible);
    if !ok {
        return Err(format!("no scene entity `{}`", a.name));
    }
    Ok(Outcome::Done(json!({}))) 
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LocateArgs {
    /// `solid` | `face` | `edge` | `vertex`
    pub kind: String,
    /// The entity name (a vertex takes its topoId as a number string).
    pub name: String,
}

/// The ARC-LENGTH midpoint of a display polyline — half the accumulated chord
/// length in from either end, interpolated inside the segment that straddles it.
///
/// Not `pl[pl.len() / 2]`, which this used to be. A display polyline is sampled
/// to a chord tolerance, so a STRAIGHT edge carries exactly two points and the
/// index midpoint of two points is `pl[1]`: an ENDPOINT. Every straight edge of
/// a box therefore anchored on a corner, three edges answered with the same
/// point, and `click_entity` on any of them reported the vertex — or another
/// edge — in the way. Arc length also beats the index midpoint on a sampled
/// curve, where the samples are denser through the bends.
///
/// `None` only for an empty polyline. A zero-length one answers with its first
/// point.
fn polyline_midpoint(polyline: &[[f32; 3]]) -> Option<[f64; 3]> {
    let point = |i: usize| -> [f64; 3] {
        let p = polyline[i];
        [p[0] as f64, p[1] as f64, p[2] as f64]
    };
    if polyline.is_empty() {
        return None;
    }
    let seg = |i: usize| -> f64 {
        let (a, b) = (point(i), point(i + 1));
        ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
    };
    let total: f64 = (0..polyline.len().saturating_sub(1)).map(seg).sum();
    if total <= 0.0 {
        return Some(point(0));
    }
    let mut walked = 0.0;
    for i in 0..polyline.len() - 1 {
        let len = seg(i);
        if walked + len >= total / 2.0 {
            let t = if len > 0.0 { (total / 2.0 - walked) / len } else { 0.0 };
            let (a, b) = (point(i), point(i + 1));
            return Some([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]);
        }
        walked += len;
    }
    Some(point(polyline.len() - 1))
}

/// The world point that stands for an entity: a face's display-triangle
/// centroid, an edge's arc-length midpoint, a vertex's position, a solid's
/// bounding-box centre.
///
/// A committed SKETCH is a scene solid too (`SolidDisplay::is_sketch`), and it
/// is the one display that may carry NO MESH: an open chain draws only its
/// segments, a hole-placement sketch only its points, and a Helix draws one
/// edge. Those are exactly the things a Revolve axis, a Sweep path, a Tube path
/// and a Hole placement are picked from, and `kind: "solid"` used to refuse
/// every one of them with "has no display mesh". So a mesh-less display falls
/// back to a point that is ON its geometry — the midpoint of its longest edge,
/// else its first vertex — rather than to a bounding-box centre, which for an
/// open chain need not lie on the chain at all.
fn anchor_point(scene: &brep_render::scene::RenderScene, kind: &str, name: &str) -> Result<([f64; 3], String), String> {
    match kind {
        "solid" => {
            let solid = scene.solid(name).ok_or_else(|| format!("no solid `{name}`"))?;
            let pos = &solid.mesh.positions;
            if pos.is_empty() {
                // See the note above: a mesh-less sketch sheet anchors ON its
                // own geometry, not at the centre of a box around it.
                let longest = solid
                    .edges
                    .iter()
                    .filter(|e| e.polyline.len() >= 2)
                    .max_by(|a, b| {
                        let span = |e: &brep_render::scene::EdgeDisplay| {
                            e.polyline.windows(2).map(|w| {
                                let (p, q) = (w[0], w[1]);
                                (((q[0] - p[0]) as f64).powi(2) + ((q[1] - p[1]) as f64).powi(2) + ((q[2] - p[2]) as f64).powi(2)).sqrt()
                            }).sum::<f64>()
                        };
                        span(a).total_cmp(&span(b))
                    });
                if let Some(point) = longest.and_then(|e| polyline_midpoint(&e.polyline)) {
                    return Ok((point, solid.name.clone()));
                }
                if let Some(vertex) = solid.vertices.first() {
                    return Ok((vertex.position, solid.name.clone()));
                }
                return Err(format!("solid `{name}` has no display mesh, no edge and no vertex to stand for it"));
            }
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for p in pos {
                for i in 0..3 {
                    lo[i] = lo[i].min(p[i] as f64);
                    hi[i] = hi[i].max(p[i] as f64);
                }
            }
            Ok(([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0], name.to_string()))
        }
        "face" => {
            for solid in scene.solids() {
                if let Some(face) = solid.faces.iter().find(|f| f.name == name) {
                    let idx = &solid.mesh.indices;
                    let pos = &solid.mesh.positions;
                    let start = (face.tri_start as usize) * 3;
                    let end = ((face.tri_start + face.tri_count) as usize * 3).min(idx.len());
                    if end <= start {
                        return Err(format!("face `{name}` has no triangles"));
                    }
                    let mut acc = [0.0f64; 3];
                    let mut n = 0.0;
                    for &i in &idx[start..end] {
                        if let Some(p) = pos.get(i as usize) {
                            for k in 0..3 {
                                acc[k] += p[k] as f64;
                            }
                            n += 1.0;
                        }
                    }
                    return Ok(([acc[0] / n, acc[1] / n, acc[2] / n], solid.name.clone()));
                }
            }
            Err(format!("no face `{name}`"))
        }
        "edge" => {
            for solid in scene.solids() {
                if let Some(edge) = solid.edges.iter().find(|e| e.name == name) {
                    let point = polyline_midpoint(&edge.polyline)
                        .ok_or_else(|| format!("edge `{name}` has no polyline"))?;
                    return Ok((point, solid.name.clone()));
                }
            }
            Err(format!("no edge `{name}`"))
        }
        "vertex" => {
            let id: u64 = name.parse().map_err(|_| format!("vertex name must be a topoId number, got `{name}`"))?;
            for solid in scene.solids() {
                if let Some(v) = solid.vertices.iter().find(|v| v.topo_id == id) {
                    return Ok((v.position, solid.name.clone()));
                }
            }
            Err(format!("no vertex with topoId {id}"))
        }
        other => Err(format!("kind `{other}` must be solid | face | edge | vertex")),
    }
}

/// Is this pick candidate the ENTITY that was located?
///
/// The candidate's `name` alone cannot answer it. A solid's anchor is its bbox
/// centre, where what the pointer meets is one of its OWN faces; a vertex
/// candidate carries no name at all, only its owning solid. Comparing `name`
/// for every kind — which this used to do — reported a solid and a vertex as
/// covered by themselves, exactly the two kinds a caller most wants to click.
fn candidate_is(candidate: &Value, kind: &str, name: &str, owner: &str) -> bool {
    let field = |k: &str, want: &str| candidate.get(k).and_then(Value::as_str) == Some(want);
    // The OWNING SOLID standing in for one of its parts. The selection filter
    // decides what a click may resolve to, so a reference field that takes
    // SOLIDs offers only solids under the pointer: pressing on a cylinder's top
    // face there picks the cylinder, and that is the click landing on the face,
    // not a click going somewhere else.
    let owning_solid = field("kind", "SOLID") && (field("name", owner) || field("solid", owner));
    match kind {
        // Any face, edge or vertex OF that solid reaches the solid.
        "solid" => field("name", name) || field("solid", name),
        // Vertex candidates are nameless: the owning solid plus the kind is all
        // there is, and `VERTEX_PICK_PX` already bounds how far the pick may be
        // from the anchor.
        "vertex" => (field("kind", "VERTEX") && field("solid", owner)) || owning_solid,
        _ => field("name", name) || owning_solid,
    }
}

fn locate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: LocateArgs = parse_args(args)?;
    let view = ctx.app.viewport.last_rect().ok_or("the 3D viewport was not drawn last frame")?;
    let engine = ctx.app.docs.engine();
    let (world, owner) = anchor_point(&engine.scene, &a.kind, &a.name)?;
    let projected = engine.world_to_screen_json(&json!([world]).to_string())?;
    let pts: Vec<[f64; 4]> = serde_json::from_str(&projected).map_err(|e| format!("projection parse: {e}"))?;
    let p = pts.first().ok_or("projection returned nothing")?;
    let (x, y) = (view.min.x as f64 + p[0], view.min.y as f64 + p[1]);
    let visible = p[3] > 0.5;
    // What a click at (x, y) would actually CONSIDER: the ranked, SELECTION-
    // FILTER-respecting candidate list the viewport's own click path builds —
    // construction planes included, filtered-off kinds excluded. The raw scene
    // pick would answer a different question: it reports an edge the live
    // filter forbids, and no plane at all.
    let under: Value = parse(&engine.candidates_at(p[0], p[1]));
    let list = under.as_array().cloned().unwrap_or_default();
    // Where this entity sits in that list. Rank 0 is what a click there
    // resolves to; anything else is something in front of it (and a reference
    // field's picker takes the top candidate outright — there is no pick-list
    // popup in that mode to choose from). `null` means it is not under the
    // pointer at all.
    let rank = list.iter().position(|c| candidate_is(c, &a.kind, &a.name, &owner));
    // …and what a REGULAR click there actually TAKES, which since the selection
    // UX changed is a different question from "the top row of that list". A
    // plain click resolves to the NEAREST admitted candidate by depth
    // (`nearest_candidate_at`); the list's own category-major order — an
    // occluded edge ahead of the face in front of it — is what a dwell, or an
    // Alt+click, opens. In REFERENCE-SELECTION mode none of that applies: the
    // field's picker still takes the top candidate outright, unchanged, so the
    // list's head is the honest answer there.
    let top = list.first().cloned().unwrap_or(Value::Null);
    let takes = if engine.ref_select_active() {
        top.clone()
    } else {
        parse(&engine.nearest_candidate_json(p[0], p[1]))
    };
    // Whether a click at this point REACHES the entity that was named. This, not
    // `rank`, is what `click_entity` refuses on: a click that would land on
    // something else is an error rather than a silent wrong pick.
    let reaches = takes
        .as_object()
        .is_some_and(|_| candidate_is(&takes, &a.kind, &a.name, &owner));
    Ok(Outcome::Done(json!({
        "kind": a.kind,
        "name": a.name,
        "x": x,
        "y": y,
        "depth": p[2],
        "visible": visible,
        "world": world,
        "owner": owner,
        "under": under,
        "rank": rank,
        "takes": takes.clone(),
        "reaches": reaches,
        "occluded_by": if reaches { Value::Null } else { takes },
    })))
}

fn mass_properties(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MassArgs = parse_args(args)?;
    let v = parse(&ctx.app.docs.engine().mass_properties_json(a.name.as_deref(), a.density));
    if v.get("ok") == Some(&Value::Bool(false)) {
        return Err(v.get("message").and_then(Value::as_str).unwrap_or("mass properties unavailable").to_string());
    }
    Ok(Outcome::Done(v))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "scene_entities", group: "scene", doc: "Every resident solid with its face and edge names and vertex positions (the reference names feature parameters take), plus the scene listing.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(scene_entities) },
    CommandSpec { name: "pick", group: "scene", doc: "Ranked pick candidates under a surface point (VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT). Does not change the selection.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<PointArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pick) },
    CommandSpec { name: "hover", group: "scene", doc: "Set the hover highlight to whatever is under a surface point and return it.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<PointArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(hover) },
    CommandSpec { name: "select", group: "scene", doc: "Replace the selection with one named entity: kind solid | face | edge | datum.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SelectArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(select) },
    CommandSpec { name: "selection_set", group: "scene", doc: "Replace the WHOLE selection with these named entities — the write twin of `selection`, and the way to put a multi-entity selection back (`select` takes one name). Vertices have no names and are not restored.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SelectionSetArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(selection_set) },
    CommandSpec { name: "select_clear", group: "scene", doc: "Clear the selection.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(select_clear) },
    CommandSpec { name: "selection", group: "scene", doc: "The current selection `{solids, faces, edges, datums, vertices}`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(selection) },
    CommandSpec { name: "set_visible", group: "scene", doc: "Show or hide a scene entity by name.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<VisibleArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(set_visible) },
    CommandSpec { name: "locate", group: "scene", doc: "Where an entity is on the surface, in egui points: a face's triangle centroid, an edge's ARC-LENGTH midpoint, a vertex (by topoId), a solid's bbox centre — projected through the camera. A committed SKETCH is reached as `kind: \"solid\"` under its feature id; one with no mesh (an open chain, a points-only hole placement, a Helix) anchors on the midpoint of its longest edge, or on its first point. `under` is the ranked candidate list a DWELL there would open, filtered the way the viewport filters it; `rank` is this entity's place in that list. What a REGULAR click takes is `takes` — the nearest admitted candidate by depth, which is also what the hover highlight lights — and `reaches` says whether that is this entity; `occluded_by` is what the click would take instead. (A reference field's picker is unchanged and still takes the list's top candidate, so `takes` is that while one is open.) The list resolves a part to its OWNING SOLID where the filter admits only solids, so a face of a cylinder ranks 0 when the cylinder does. The bridge from a reference name to a pointer click, which `click_entity` and `drag_entity` spend.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<LocateArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(locate) },
    CommandSpec { name: "mass_properties", group: "scene", doc: "Volume, surface area, mass, centroid, inertia and principal moments of one solid or of all — the closed-form check for an example model.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<MassArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(mass_properties) },
];

