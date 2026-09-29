//! Sheet-metal fillets and symmetric chamfers of convex plate corners.
//!
//! Treatments update the flat outline in [`SheetTree`], so folded geometry and
//! flat-pattern exports share the result. A vertical thickness edge selects one
//! corner; a top or bottom face selects all treatable corners on that flat.
//! Two adjacent side faces also select their shared corner. Selections are
//! deduplicated; whole-face picks skip corners that cannot be treated.
//!
//! Both adjacent outline segments must be straight and free of bends. The
//! setback must be shorter than both current segments, including shortening by
//! earlier treatments, to prevent overlapping corners. Reflex and collinear
//! corners are rejected when selected explicitly.

use std::collections::{BTreeMap, BTreeSet};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::sheet_metal::{self, Edge, Flat, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_arc, Vec3};

/// A corner is convex only when the CCW outline turns left by at least this much
/// (in the 2D cross product of the two edge directions); at/below it the corner is
/// reflex or collinear and refused.
const TURN_EPS: f64 = 1e-9;

/// Which corner treatment a run applies.
#[derive(Clone, Copy)]
enum Kind {
    /// A circular arc of `param` = radius, tangent to both segments.
    Fillet,
    /// A straight bevel set back `param` = distance along both segments.
    Chamfer,
}

pub fn execute_fillet(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx, Kind::Fillet) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

pub fn execute_chamfer(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx, Kind::Chamfer) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext, kind: Kind) -> Result<FeatureResult, String> {
    let what = match kind {
        Kind::Fillet => "sheet-metal corner fillet",
        Kind::Chamfer => "sheet-metal corner chamfer",
    };

    let Some(body_name) = common::sheet_body_name(ctx) else {
        return Ok(FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone()));
    };
    let Some(handle) = ctx.scene.resolve_solid(&body_name) else {
        let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
        result.unresolved.push(body_name);
        return Ok(result);
    };
    let Some(mut tree) = sheet_metal::get_tree(handle) else {
        return Err(format!("{what}: target '{body_name}' is not a sheet-metal body"));
    };

    // --- The treatment size ---
    let param = match kind {
        Kind::Fillet => ctx.number("radius")?,
        Kind::Chamfer => ctx.number("distance")?,
    };
    if !(param > 0.0) {
        let field = match kind {
            Kind::Fillet => "radius",
            Kind::Chamfer => "distance",
        };
        return Err(format!("{what}: {field} must be positive, got {param}"));
    }

    // --- Resolve the picked corners to (flat_id, vertex_index) ---
    let picks = common::reference_names(ctx.param("corners"));
    if picks.is_empty() {
        return Err(format!(
            "{what}: missing `corners` selection — pick a corner EDGE (between two plate side \
             faces) for one corner, or a plate top/bottom FACE for all of its corners"
        ));
    }
    let corners = resolve_corners(&tree, &picks, what)?;

    // --- Apply the surgery, flat by flat, high vertex index first (so an insert
    //     never disturbs a lower pending corner; a shared segment is validated
    //     against the already-shortened neighbour) ---
    let mut counter = 0usize;
    for (flat_id, vertices) in &corners {
        let flat = tree.root.find_by_id_mut(flat_id)
            .ok_or_else(|| format!("{what}: flat '{flat_id}' vanished from the tree"))?;
        for &vertex in vertices.iter().rev() {
            let new_id = format!("{}:c{counter}", ctx.id);
            counter += 1;
            apply_corner(flat, vertex, kind, param, &new_id, what)?;
        }
    }

    // --- Re-evaluate + replace the body in place (mirrors SM.CUTOUT) ---
    let added = sheet_metal::register_folded(tree, &body_name)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added.push(added);
    result.removed.push(body_name);
    Ok(result)
}

// ===========================================================================
// Selection resolution
// ===========================================================================

/// One SIDE-face reference: the flat it lives on and the edge (outline segment)
/// id it walls.
struct SideRef {
    flat_id: String,
    edge_id: String,
}

/// Parse the picked references into corners: `flat_id -> {corner vertex index}`
/// (ascending BTreeSets so the apply loop can walk them in reverse).
fn resolve_corners(
    tree: &SheetTree,
    picks: &[String],
    what: &str,
) -> Result<BTreeMap<String, BTreeSet<usize>>, String> {
    // Corners accumulate here from BOTH pick kinds (deduped per flat by the set):
    // a plate top/bottom FACE pick expands directly to all treatable corners; a
    // SIDE-face / corner-EDGE pick contributes segment refs that pair by adjacency.
    let mut corners: BTreeMap<String, BTreeSet<usize>> = BTreeMap::new();
    // Each pick contributes 1 or 2 SIDE-face refs (a bare side face, or the `A|B`
    // corner-edge string joining the two adjacent walls).
    let mut refs: Vec<SideRef> = Vec::new();
    for pick in picks {
        // A STANDALONE plate cap face (`{flat}:A` / `{flat}:B`, no `|`) expands to
        // ALL of that flat's treatable corners. A `|` pick is the corner-edge
        // overlay whose halves are SIDE faces — a rim edge (`SIDE|A`) still rejects
        // through `parse_side_ref`, so cap expansion is gated on a bare pick.
        if !pick.contains('|') {
            if let Some(flat_id) = cap_face_flat(tree, pick) {
                let flat = tree.root.find_by_id(&flat_id).expect("cap flat resolved");
                let treatable = flat_treatable_corners(flat);
                if treatable.is_empty() {
                    return Err(format!(
                        "{what}: plate face '{flat_id}' has no treatable corner — every outline \
                         vertex is reflex, on a bend line, or curved"
                    ));
                }
                corners.entry(flat_id).or_default().extend(treatable);
                continue;
            }
        }
        for token in pick.split('|') {
            refs.push(parse_side_ref(token, what)?);
        }
    }

    // The set of SELECTED outline-segment indices per flat (SIDE-face picks only).
    let mut selected: BTreeMap<String, BTreeSet<usize>> = BTreeMap::new();
    for side in &refs {
        let flat = tree.root.find_by_id(&side.flat_id).ok_or_else(|| {
            format!(
                "{what}: face '{}:SIDE:{}' names flat '{}', which is not part of this body",
                side.flat_id, side.edge_id, side.flat_id
            )
        })?;
        let index = flat
            .edges
            .iter()
            .position(|edge| edge.id == side.edge_id)
            .ok_or_else(|| {
                format!(
                    "{what}: flat '{}' has no outline segment '{}'",
                    side.flat_id, side.edge_id
                )
            })?;
        selected.entry(side.flat_id.clone()).or_default().insert(index);
    }

    // Two adjacent selected segments (k, (k+1)%n) share vertex (k+1)%n = a corner.
    for (flat_id, segments) in &selected {
        let flat = tree.root.find_by_id(flat_id).expect("flat resolved above");
        let n = flat.outline.len();
        let mut used: BTreeSet<usize> = BTreeSet::new();
        for &k in segments {
            let next = (k + 1) % n;
            if segments.contains(&next) {
                corners.entry(flat_id.clone()).or_default().insert(next);
                used.insert(k);
                used.insert(next);
            }
        }
        // A selected face that never pairs up is an incomplete corner pick — loud.
        if let Some(orphan) = segments.iter().find(|s| !used.contains(s)) {
            let edge_id = &flat.edges[*orphan].id;
            return Err(format!(
                "{what}: face '{flat_id}:SIDE:{edge_id}' does not meet another selected face \
                 at a corner — pick the corner EDGE, both plate side faces of the corner, or the \
                 plate top/bottom face for all corners"
            ));
        }
    }

    if corners.is_empty() {
        return Err(format!(
            "{what}: no corner found — pick a corner EDGE (between two plate side faces), both \
             side faces of a corner, or a plate top/bottom FACE for all of its corners"
        ));
    }
    Ok(corners)
}

/// If `pick` names a plate TOP/BOTTOM cap face (`{flat}:A` / `{flat}:B`, with any
/// trailing `[n]` dedup suffix), return its flat id — but ONLY when the prefix is a
/// real flat of this body. A bend BAND (`{bendId}:BAND:…:A`) or collar wall
/// (`…:wall:A`) face also ends in `:A`/`:B` but has no flat prefix, so it returns
/// `None` and falls through to the SIDE-face parser's rejection.
fn cap_face_flat(tree: &SheetTree, pick: &str) -> Option<String> {
    let name = strip_dedup_suffix(pick.trim());
    let flat_id = name.strip_suffix(":A").or_else(|| name.strip_suffix(":B"))?;
    tree.root.find_by_id(flat_id).map(|_| flat_id.to_string())
}

/// Every outline vertex of `flat` that is a TREATABLE corner: convex, with both
/// adjacent segments STRAIGHT (not curved) and carrying NO bend. Mirrors the early
/// gates of [`apply_corner`] but WITHOUT the per-size oversize check — that stays in
/// `apply_corner`, so an oversize face-picked corner still errors loudly. Reflex /
/// bend-line / curved vertices are silently skipped, so a face pick "collects each"
/// corner it can treat (including on an already-filleted plate, whose new arc
/// segments are curved and skipped).
fn flat_treatable_corners(flat: &Flat) -> Vec<usize> {
    let n = flat.outline.len();
    if n < 3 {
        return Vec::new();
    }
    (0..n).filter(|&vertex| is_treatable_corner(flat, vertex)).collect()
}

/// Whether outline `vertex` of `flat` is a convex corner between two straight,
/// unbent segments (the skip-predicate for a face pick's corner collection).
fn is_treatable_corner(flat: &Flat, vertex: usize) -> bool {
    let n = flat.outline.len();
    if n < 3 {
        return false;
    }
    let in_edge = (vertex + n - 1) % n;
    let out_edge = vertex;
    for edge_index in [in_edge, out_edge] {
        let Some(edge) = flat.edges.get(edge_index) else {
            return false;
        };
        if edge.bend.is_some() || flat.outline_curves.contains_key(&edge.id) {
            return false;
        }
    }
    let a = flat.outline[in_edge];
    let v = flat.outline[vertex];
    let b = flat.outline[(vertex + 1) % n];
    let (Some(din), Some(dout)) = (unit(sub(v, a)), unit(sub(b, v))) else {
        return false;
    };
    // Convex on a CCW loop = a LEFT turn (positive cross); reflex/collinear skipped.
    din[0] * dout[1] - din[1] * dout[0] > TURN_EPS
}

/// Parse one face token into a SIDE-face ref, refusing every non-corner face with
/// a message that names WHY (cap rim vs bore vs unknown). A trailing `[n]`
/// disambiguation suffix (from face-name dedup) is stripped.
fn parse_side_ref(token: &str, what: &str) -> Result<SideRef, String> {
    let name = strip_dedup_suffix(token.trim());
    if let Some((flat_id, edge_id)) = name.split_once(":SIDE:") {
        return Ok(SideRef {
            flat_id: flat_id.to_string(),
            edge_id: edge_id.to_string(),
        });
    }
    if name.contains(":CUTOUT:") {
        return Err(format!(
            "{what}: '{name}' is a hole/bore wall, not a plate corner — pick two adjacent \
             plate side faces"
        ));
    }
    if name.ends_with(":A") || name.ends_with(":B") {
        return Err(format!(
            "{what}: '{name}' is a top/bottom plate face (or, joined with a side face, a \
             horizontal plate rim), not a vertical plate corner — pick two adjacent plate side faces"
        ));
    }
    Err(format!(
        "{what}: '{name}' is not a plate side face — pick the two side faces meeting at a corner"
    ))
}

/// Strip a trailing `[n]` face-name dedup suffix, if present.
fn strip_dedup_suffix(name: &str) -> &str {
    match (name.rfind('['), name.ends_with(']')) {
        (Some(open), true) if name[open + 1..name.len() - 1].chars().all(|c| c.is_ascii_digit()) => {
            &name[..open]
        }
        _ => name,
    }
}

// ===========================================================================
// Outline surgery
// ===========================================================================

/// Replace the sharp corner at outline `vertex` of `flat` with an arc (fillet) or
/// bevel (chamfer), keeping `outline` / `edges` / `outline_curves` consistent.
///
/// Given corner V with incoming segment A→V (edge index `c-1`) and outgoing V→B
/// (edge index `c`), the surgery sets `outline[c] = T1` (tangent point on A→V),
/// inserts `T2` (tangent point on V→B) right after it, and splices a NEW segment
/// `T1→T2` (the arc/bevel) at edge index `c` — so the old outgoing edge keeps its
/// id on the shifted `T2→B` segment and the incoming edge keeps its id on the
/// shortened `A→T1` segment.
fn apply_corner(
    flat: &mut Flat,
    vertex: usize,
    kind: Kind,
    param: f64,
    new_id: &str,
    what: &str,
) -> Result<(), String> {
    let n = flat.outline.len();
    if n < 3 {
        return Err(format!("{what}: flat '{}' has a degenerate outline", flat.id));
    }
    let c = vertex;
    let prev_v = (c + n - 1) % n;
    let next_v = (c + 1) % n;
    let in_edge = (c + n - 1) % n; // segment A -> V
    let out_edge = c; // segment V -> B

    // Both adjacent segments must be STRAIGHT and carry no fold (v1 scope).
    for (edge_index, role) in [(in_edge, "incoming"), (out_edge, "outgoing")] {
        let edge = &flat.edges[edge_index];
        if edge.bend.is_some() {
            return Err(format!(
                "{what}: corner at flat '{}' has a folded {role} edge '{}' — corners on a bend \
                 line (shared across flats) are out of scope; treat single-flat corners only",
                flat.id, edge.id
            ));
        }
        if flat.outline_curves.contains_key(&edge.id) {
            return Err(format!(
                "{what}: corner at flat '{}' has a curved {role} edge '{}' — only corners between \
                 two straight segments can be treated",
                flat.id, edge.id
            ));
        }
    }

    let a = flat.outline[prev_v];
    let v = flat.outline[c];
    let b = flat.outline[next_v];

    // Edge directions along the CCW traversal.
    let din = unit(sub(v, a)).ok_or_else(|| degenerate(what, flat, "incoming"))?; // A -> V
    let dout = unit(sub(b, v)).ok_or_else(|| degenerate(what, flat, "outgoing"))?; // V -> B
    // Convex on a CCW loop = a LEFT turn (positive cross); refuse reflex/collinear.
    let turn = din[0] * dout[1] - din[1] * dout[0];
    if turn <= TURN_EPS {
        return Err(format!(
            "{what}: corner at flat '{}' is reflex or straight (interior angle >= 180°) — only \
             convex corners are supported",
            flat.id
        ));
    }

    // The two rays leaving V, and the half interior angle between them.
    let u1 = [-din[0], -din[1]]; // toward A
    let u2 = dout; // toward B
    let cos_phi = (u1[0] * u2[0] + u1[1] * u2[1]).clamp(-1.0, 1.0);
    let phi = cos_phi.acos();
    let half = phi * 0.5;

    let setback = match kind {
        // Tangent length of a radius-`param` arc: r / tan(half interior angle).
        Kind::Fillet => param / half.tan(),
        // Symmetric bevel: the distance back along each edge is `param` directly.
        Kind::Chamfer => param,
    };

    // Oversize guard: strictly shorter than BOTH live adjacent segments (so a
    // shared segment already shortened by a neighbour corner is caught too).
    let in_len = length(sub(v, a));
    let out_len = length(sub(b, v));
    let eps = 1e-9 * in_len.max(out_len).max(1.0);
    if setback >= in_len - eps || setback >= out_len - eps {
        let (field, value) = match kind {
            Kind::Fillet => ("radius", param),
            Kind::Chamfer => ("distance", param),
        };
        return Err(format!(
            "{what}: {field} {value} is too large for the corner at flat '{}' — its {setback:.6} \
             setback exceeds an adjacent segment ({in_len:.6} / {out_len:.6}); it would \
             self-intersect the outline",
            flat.id
        ));
    }

    let t1 = [v[0] + u1[0] * setback, v[1] + u1[1] * setback];
    let t2 = [v[0] + u2[0] * setback, v[1] + u2[1] * setback];

    // Build the connecting segment's exact curve (fillet only; a chamfer's straight
    // bevel needs no override).
    let curve = match kind {
        Kind::Fillet => {
            // Arc centre on the interior bisector, radius `param`.
            let bis = unit([u1[0] + u2[0], u1[1] + u2[1]])
                .ok_or_else(|| format!("{what}: degenerate corner bisector at flat '{}'", flat.id))?;
            let center_2d = [v[0] + bis[0] * param / half.sin(), v[1] + bis[1] * param / half.sin()];
            let center = Vec3::new(center_2d[0], center_2d[1], 0.0);
            let x_axis = Vec3::new(t1[0] - center_2d[0], t1[1] - center_2d[1], 0.0);
            // +Z is the outline's up-normal (CCW), so n × x gives a CCW y-axis; the
            // convex fillet then sweeps CCW from T1 to T2, bulging toward the corner.
            let y_axis = Vec3::new(0.0, 0.0, 1.0).cross(x_axis);
            let d2 = [t2[0] - center_2d[0], t2[1] - center_2d[1]];
            let ex = d2[0] * x_axis.x + d2[1] * x_axis.y;
            let ey = d2[0] * y_axis.x + d2[1] * y_axis.y;
            let mut end = ey.atan2(ex);
            if end <= 1e-12 {
                end += std::f64::consts::TAU;
            }
            Some(make_arc(center, x_axis, y_axis, param, 0.0, end).map_err(|error| {
                format!("{what}: could not build the corner arc at flat '{}': {error}", flat.id)
            })?)
        }
        Kind::Chamfer => None,
    };

    // Splice: V -> {T1, T2}; new segment T1->T2 at edge index c.
    flat.outline[c] = t1;
    flat.outline.insert(c + 1, t2);
    flat.edges.insert(c, Edge { id: new_id.to_string(), bend: None });
    if let Some(curve) = curve {
        flat.outline_curves.insert(new_id.to_string(), curve);
    }
    Ok(())
}

fn degenerate(what: &str, flat: &Flat, which: &str) -> String {
    format!("{what}: corner at flat '{}' has a zero-length {which} segment", flat.id)
}

// ===========================================================================
// 2D helpers
// ===========================================================================

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
fn length(d: [f64; 2]) -> f64 {
    (d[0] * d[0] + d[1] * d[1]).sqrt()
}
fn unit(d: [f64; 2]) -> Option<[f64; 2]> {
    let len = length(d);
    (len > 1e-12).then(|| [d[0] / len, d[1] / len])
}

// ===========================================================================
// Schemas (kernel-owned)
// ===========================================================================

/// Shared `corners` reference_selection: a vertical corner EDGE (one corner), a
/// plate top/bottom FACE (all of its corners), or the two adjacent SIDE faces.
fn corners_param() -> serde_json::Value {
    serde_json::json!({
        "type": "reference_selection",
        "selectionFilter": ["FACE", "EDGE"],
        "multiple": true,
        "default_value": null,
        "hint": "Pick a corner EDGE (between two plate side faces) to treat that one corner, or a plate top/bottom FACE to treat all of that plate's corners. Picking the two adjacent side faces of a corner also works; mix edges and faces freely."
    })
}

fn sheet_param() -> serde_json::Value {
    serde_json::json!({
        "type": "reference_selection",
        "selectionFilter": ["SOLID"],
        "multiple": false,
        "default_value": null,
        "hint": "Target sheet-metal body. Optional: auto-resolves when there is exactly one sheet-metal body."
    })
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces/edges drive `corners`, a solid `sheet`
/// (one predicate for both the SM.FILLET and SM.CHAMFER catalogue entries).
/// Both round/chamfer the CORNERS of an existing sheet-metal flat, so the
/// selection must sit on a sheet-metal body (`all_sheet_metal`) — plain geometry
/// does not offer them.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.all_sheet_metal && (probe.faces > 0 || probe.edges > 0 || probe.solids > 0)
}

pub fn schema_fillet() -> serde_json::Value {
    serde_json::json!({
        "type": "SM.FILLET",
        "shortName": "SM.CFIL",
        "longName": "SM Corner Fillet",
        "displayBuilder": false,
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null, "hint": "Unique identifier for the corner fillet" },
            "sheet": sheet_param(),
            "corners": corners_param(),
            "radius": {
                "type": "number",
                "default_value": 1,
                "min": 0,
                "hint": "Corner fillet radius (a circular arc tangent to both adjacent outline segments)."
            }
        }
    })
}

pub fn schema_chamfer() -> serde_json::Value {
    serde_json::json!({
        "type": "SM.CHAMFER",
        "shortName": "SM.CCHM",
        "longName": "SM Corner Chamfer",
        "displayBuilder": false,
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null, "hint": "Unique identifier for the corner chamfer" },
            "sheet": sheet_param(),
            "corners": corners_param(),
            "distance": {
                "type": "number",
                "default_value": 1,
                "min": 0,
                "hint": "Corner chamfer setback distance (a symmetric bevel, distance-distance, back along both adjacent segments)."
            }
        }
    })
}

