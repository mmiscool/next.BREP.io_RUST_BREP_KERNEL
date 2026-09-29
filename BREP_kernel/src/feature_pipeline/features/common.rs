//! Shared feature parameters, reference resolution, and solid operations.

use rustc_hash::FxHashSet;

use crate::feature_pipeline::{
    FeatureRefusal,
    AddedSolid, Axis, EdgeRef, FaceRef, FeatureContext, FeatureResult, Frame, SketchProfile,
};
use crate::{AffineTransform, BooleanOperation, BooleanOptions, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3, transform_brep};

/// Rotate a solid a half turn (180°) about the local X axis. This is a PROPER
/// rotation (determinant +1), so `transform_brep` preserves face winding and every
/// stamped face name. It realizes a NEGATIVE directional dimension: a primitive
/// built along +Y with the value's MAGNITUDE is spun to occupy the -Y side, and a
/// partial revolve/torus sweep is spun to the opposite side — exactly the geometry
/// a signed value implies. `(x, y, z) -> (x, -y, -z)`.
pub fn rotate_half_turn_about_x(solid: BrepSolid) -> Result<BrepSolid, String> {
    let half_turn = AffineTransform::new([
        1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ])?;
    transform_brep(&solid, half_turn, false)
}

/// Collect `(face_id, name)` for every NAMED face, in shell/face order.
pub fn collect_face_names(solid: &BrepSolid) -> Vec<(u64, String)> {
    let mut names = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if let Some(name) = &face.name {
                names.push((face.id, name.clone()));
            }
        }
    }
    names
}

/// Collect `(edge_id, name)` for every NAMED edge.
pub fn collect_edge_names(solid: &BrepSolid) -> Vec<(u64, String)> {
    solid
        .edges
        .iter()
        .filter_map(|edge| edge.name.as_ref().map(|name| (edge.id, name.clone())))
        .collect()
}

/// Stamp one face's operation-role metadata (`faceRole` + `operationFaceType`,
/// the established Sweep/Revolve convention the PMI dimension UI and tests read) into the
/// kernel scene-metadata store, keyed by face NAME so it follows the face
/// through booleans. Only the role keys are stamped — geometric metadata
/// (`faceType`/`surfaceType`) stays inferred from the surface on the read side.
pub fn stamp_face_role(face_name: &str, role: &str, op_type: &str) {
    let mut record = serde_json::Map::new();
    record.insert("faceRole".into(), serde_json::Value::String(role.into()));
    record.insert(
        "operationFaceType".into(),
        serde_json::Value::String(op_type.into()),
    );
    crate::feature_pipeline::scene_metadata::merge_record(face_name, &record, true);
}

/// Stamp the whole sweep-family role convention on a BUILT solid's named faces:
/// a name in `start_names`/`end_names` → `start_cap`/`STARTCAP` +
/// `end_cap`/`ENDCAP`, any other name ending in `wall_suffix` →
/// `sidewall`/`SIDEWALL`. Walking the actual faces (not the intended name list)
/// means an absent cap (full revolve) or a skipped axis wall never leaves a
/// phantom record in the store.
///
/// The lists are per-loop: each region of a multi-loop profile names its own
/// caps (`{cap_base}:L{loopId}_START`), so the role pass takes the whole list
/// rather than one name per end. A single-loop profile passes a one-element
/// slice — there was a `stamp_sweep_roles` wrapper for that, and every caller
/// went through this function instead, so it is gone.
pub fn stamp_sweep_roles_multi(
    solid: &BrepSolid,
    wall_suffix: &str,
    start_names: &[String],
    end_names: &[String],
) {
    for face in solid.shells.iter().flat_map(|shell| shell.faces.iter()) {
        let Some(name) = face.name.as_deref() else {
            continue;
        };
        if start_names.iter().any(|start| start == name) {
            stamp_face_role(name, "start_cap", "STARTCAP");
        } else if end_names.iter().any(|end| end == name) {
            stamp_face_role(name, "end_cap", "ENDCAP");
        } else if name.ends_with(wall_suffix) {
            stamp_face_role(name, "sidewall", "SIDEWALL");
        }
    }
}

/// The per-loop cap names of one profile-swept feature, plus the CONTAINER names
/// they roll up into — the single place the per-loop cap convention is spelled,
/// shared by extrude/sweep/revolve/path-sweep/loft so the five stay byte-identical.
///
/// `{cap_base}:{loop key}_START` / `_END` per region (see [`ProfileLoop::key`]),
/// with `{cap_base}_START` / `_END` as the containers standing for all of them.
/// A SINGLE-loop profile is the special case that matters most for compatibility:
/// its one cap keys off its loop like any other, and its container has exactly one
/// member, so every reference stored before per-loop naming resolves to exactly
/// the face it always meant.
pub struct CapNames {
    /// Per region, in region order: `(start, end)`.
    pub per_loop: Vec<(String, String)>,
    pub start_container: String,
    pub end_container: String,
}

impl CapNames {
    /// Build the cap names for `regions` under `cap_base`.
    ///
    /// A region whose loop has no identity to key on (a FACE profile, a
    /// hand-built profile) keeps the un-keyed `{cap_base}_START/_END` spelling it
    /// has always had — such a profile is single-region, so there is nothing to
    /// disambiguate and every saved reference to it stays exact.
    pub fn new(cap_base: &str, regions: &[Vec<crate::feature_pipeline::ProfileLoop>]) -> Self {
        let per_loop = regions
            .iter()
            .map(|region| {
                match region.first().and_then(|outer| outer.key()) {
                    Some(key) => (
                        format!("{cap_base}:{key}_START"),
                        format!("{cap_base}:{key}_END"),
                    ),
                    None => (format!("{cap_base}_START"), format!("{cap_base}_END")),
                }
            })
            .collect();
        Self {
            per_loop,
            start_container: format!("{cap_base}_START"),
            end_container: format!("{cap_base}_END"),
        }
    }

    /// This feature's `(container, members)` pairs for [`register_added_grouped`].
    /// A container whose only member IS the container name (the un-keyed
    /// single-region case) is dropped — the face is reachable exactly, so there is
    /// no group to register.
    pub fn containers(&self) -> Vec<(String, Vec<String>)> {
        [
            (self.start_container.clone(), self.starts()),
            (self.end_container.clone(), self.ends()),
        ]
        .into_iter()
        .filter(|(container, members)| members.as_slice() != [container.clone()])
        .collect()
    }

    /// The INTERIOR cap names of a multi-segment sweep: one path SEGMENT's own
    /// pair for `region_index`, spelled `{cap_base}:{loop key}:{segment}_START/_END`.
    ///
    /// A sweep along several path segments builds one portion per segment, and
    /// every portion has two caps. The chain's own two ends keep the plain
    /// per-loop spelling ([`Self::per_loop`]); every cap BETWEEN two portions is
    /// a copy of the profile that the neighbouring portion's opposite cap cancels
    /// in the union, so it normally does not reach the result at all. Naming it
    /// after its segment anyway means that if a path ever leaves one standing
    /// (a chain that doubles back), it arrives with a name tied to the segment
    /// that built it instead of a positional `[n]` disambiguation.
    pub fn interior(&self, region_index: usize, segment: &str) -> (String, String) {
        let (start, end) = &self.per_loop[region_index];
        let start_base = start.strip_suffix("_START").unwrap_or(start);
        let end_base = end.strip_suffix("_END").unwrap_or(end);
        (
            format!("{start_base}:{segment}_START"),
            format!("{end_base}:{segment}_END"),
        )
    }

    /// All per-loop START names, for [`stamp_sweep_roles_multi`].
    pub fn starts(&self) -> Vec<String> {
        self.per_loop.iter().map(|(start, _)| start.clone()).collect()
    }

    /// All per-loop END names, for [`stamp_sweep_roles_multi`].
    pub fn ends(&self) -> Vec<String> {
        self.per_loop.iter().map(|(_, end)| end.clone()).collect()
    }
}

pub(super) fn parse_boolean_operation(operation: &str) -> Result<BooleanOperation, String> {
    match operation {
        "UNION" => Ok(BooleanOperation::Union),
        "SUBTRACT" => Ok(BooleanOperation::Subtract),
        "INTERSECT" => Ok(BooleanOperation::Intersect),
        other => Err(format!("unsupported boolean operation '{other}'")),
    }
}

/// The optional `boolean` param, parsed from `inputParams.boolean`.
struct BooleanParam {
    operation: String,
    targets: Vec<String>,
    merge_coplanar_faces: bool,
}

fn read_boolean_param(ctx: &FeatureContext) -> BooleanParam {
    let boolean = ctx.param("boolean");
    let operation = boolean
        .and_then(|value| value.get("operation"))
        .and_then(|value| value.as_str())
        .unwrap_or("NONE")
        .to_uppercase();
    let targets = boolean
        .and_then(|value| value.get("targets"))
        .and_then(|value| value.as_array())
        .map(|array| {
            array
                .iter()
                .filter_map(|entry| entry.as_str())
                .map(|name| name.trim().to_string())
                .filter(|name| !name.is_empty())
                .collect()
        })
        .unwrap_or_default();
    let merge_coplanar_faces = boolean
        .and_then(|value| value.get("mergeCoplanarFaces"))
        .and_then(|value| value.as_bool())
        .unwrap_or(true);
    BooleanParam {
        operation,
        targets,
        merge_coplanar_faces,
    }
}

/// Collect up to two distinct face names per edge, preserving first encounter order.
fn collect_edge_face_names(
    solid: &BrepSolid,
) -> (std::collections::HashMap<u64, Vec<String>>, Vec<u64>) {
    use std::collections::HashMap;
    let mut edge_faces: HashMap<u64, Vec<String>> = HashMap::new();
    let mut encounter_order: Vec<u64> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            let face_name = face.name.clone().unwrap_or_default();
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    let entry = edge_faces.entry(coedge.edge_id).or_insert_with(|| {
                        encounter_order.push(coedge.edge_id);
                        Vec::new()
                    });
                    if entry.len() < 2 && !entry.contains(&face_name) {
                        entry.push(face_name.clone());
                    }
                }
            }
        }
    }
    (edge_faces, encounter_order)
}

/// Name topology edges `{faceA}|{faceB}[n]`, sorting adjacent face names and
/// numbering each pair in shell/face/loop encounter order. Existing names that
/// contain `|` are regenerated; authored names are preserved.
pub fn stamp_derived_edge_names(solid: &mut BrepSolid) {
    use std::collections::HashMap;
    let (edge_faces, encounter_order) = collect_edge_face_names(solid);
    let solid_name_fallback = "Solid".to_string();
    let mut base_counts: HashMap<String, usize> = HashMap::new();
    for edge_id in encounter_order {
        let Some(edge) = solid.edges.iter_mut().find(|edge| edge.id == edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let mut faces: Vec<String> = edge_faces
            .get(&edge_id)
            .map(|list| list.iter().filter(|n| !n.is_empty()).cloned().collect())
            .unwrap_or_default();
        faces.sort();
        let base = if faces.len() >= 2 {
            format!("{}|{}", faces[0], faces[1])
        } else {
            format!(
                "{}|BOUNDARY",
                faces.first().unwrap_or(&solid_name_fallback)
            )
        };
        // Only topology edges advance the counter, keeping each pair's indices dense.
        let is_topology_name = edge.name.as_deref().is_none_or(|n| n.contains('|'));
        if is_topology_name {
            let index = base_counts.entry(base.clone()).or_insert(0);
            edge.name = Some(format!("{base}[{index}]"));
            *index += 1;
        }
    }
}

/// Suffix copied face and authored-edge names with `::{suffix}`, then regenerate
/// topology-edge names from the renamed faces. This keeps copies distinct from
/// their sources in the scene's name lookup. Empty names are skipped.
pub fn namespace_copy_names(solid: &mut BrepSolid, suffix: &str) {
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if let Some(name) = face.name.as_ref() {
                let trimmed = name.trim();
                if !trimmed.is_empty() {
                    face.name = Some(format!("{trimmed}::{suffix}"));
                }
            }
        }
    }
    for edge in &mut solid.edges {
        if let Some(name) = edge.name.as_ref() {
            let trimmed = name.trim();
            // Topology edges (`|`) are re-derived below from the retagged faces;
            // authored edges take the suffix directly.
            if !trimmed.is_empty() && !trimmed.contains('|') {
                edge.name = Some(format!("{trimmed}::{suffix}"));
            }
        }
    }
    stamp_derived_edge_names(solid);
}

/// Disambiguate duplicate face names with `[n]` in shell/face encounter order.
/// Unique names stay unchanged. Run before [`stamp_derived_edge_names`] so edge
/// names embed the final face names and scene lookups remain unambiguous.
pub fn ensure_unique_face_names(solid: &mut BrepSolid) {
    use std::collections::HashMap;
    let mut counts: HashMap<String, usize> = HashMap::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if let Some(name) = &face.name {
                *counts.entry(name.clone()).or_insert(0) += 1;
            }
        }
    }
    let mut running: HashMap<String, usize> = HashMap::new();
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            let Some(name) = face.name.clone() else { continue };
            if counts.get(&name).copied().unwrap_or(0) > 1 {
                let index = running.entry(name.clone()).or_insert(0);
                face.name = Some(format!("{name}[{index}]"));
                *index += 1;
            }
        }
    }
}

/// Register a freshly built base solid as a single `AddedSolid` under `name`.
/// Deduplicates FACE names, then stamps the derived `{faceA}|{faceB}[n]` names
/// onto the edges (which embed the now-unique face names), so face AND edge
/// reference_selections both resolve uniquely against the scene.
pub fn register_added(base: BrepSolid, name: &str) -> AddedSolid {
    register_added_grouped(base, name, &[])
}

/// [`register_added`] plus CONTAINER groups: `containers` is
/// `(container_name, member_face_names)`, the un-keyed name a profile-swept
/// feature publishes for all of its per-loop faces at once (`{cap_base}_START`
/// standing for every `{cap_base}:L{id}_START`).
///
/// Members are resolved against the FINAL solid, so a face a boolean merged or
/// consumed simply drops out of its group, and a container whose members all
/// vanished registers nothing. Each container also gets the EDGE group implied by
/// it — see [`collect_edge_group_aliases`].
pub fn register_added_grouped(
    mut base: BrepSolid,
    name: &str,
    containers: &[(String, Vec<String>)],
) -> AddedSolid {
    ensure_unique_face_names(&mut base);
    stamp_derived_edge_names(&mut base);
    let face_names = collect_face_names(&base);
    let edge_names = collect_edge_names(&base);
    let face_groups = collect_face_groups(&face_names, containers);
    let mut edge_groups = collect_edge_group_aliases(&base, containers);
    collect_derived_edge_bases(&edge_names, &mut edge_groups);
    let handle = crate::register_solid_value(base);
    AddedSolid {
        handle,
        name: name.to_string(),
        face_names,
        edge_names,
        face_groups,
        edge_groups,
    }
}

/// Publish each derived edge name's BASE — `{faceA}|{faceB}` without the `[n]`
/// — as a group standing for every edge that carries it.
///
/// [`stamp_derived_edge_names`] always appends the index, even where a face
/// pair meets along a single edge, so the bare base is never an edge name and a
/// reference stored as one resolves to nothing on its own. Bare bases are
/// exactly what a model saved against an UNSTAMPED solid holds — the standalone
/// Boolean feature published the imprint's raw `{faceA}|{faceB}` until it
/// started stamping like every other feature — so they must keep naming the
/// edges they always named.
///
/// The group carries every member, which is what [`SceneMap::resolve_edge`]
/// (single) and [`SceneMap::resolve_edge_group`] (fan-out) already read
/// correctly: a base one edge carries resolves everywhere, one several edges
/// carry stays ambiguous for a single-edge consumer and fans out for a
/// selection. A base an EXACT edge name already spells is left alone (both
/// resolvers try the exact map first, so it could never be reached), and so is
/// one a container alias already claims.
fn collect_derived_edge_bases(
    edge_names: &[(u64, String)],
    groups: &mut Vec<(String, Vec<String>)>,
) {
    use std::collections::HashSet;
    let taken: HashSet<&str> = groups.iter().map(|(name, _)| name.as_str()).collect();
    let exact: HashSet<&str> = edge_names.iter().map(|(_, name)| name.as_str()).collect();
    let mut bases: Vec<(String, Vec<String>)> = Vec::new();
    for (_, name) in edge_names {
        // `{base}[{index}]`: the index is the trailing bracketed run of digits.
        let Some(base) = name.strip_suffix(']').and_then(|head| {
            let at = head.rfind('[')?;
            let (base, index) = (&head[..at], &head[at + 1..]);
            (!index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit()))
                .then_some(base)
        }) else {
            continue;
        };
        if base.is_empty() || !base.contains('|') {
            continue;
        }
        if taken.contains(base) || exact.contains(base) {
            continue;
        }
        match bases.iter_mut().find(|(existing, _)| existing == base) {
            Some((_, members)) => members.push(name.clone()),
            None => bases.push((base.to_string(), vec![name.clone()])),
        }
    }
    groups.append(&mut bases);
}

/// Keep each container's member NAMES that actually survived onto the final
/// solid. Containers with no surviving member are dropped.
fn collect_face_groups(
    face_names: &[(u64, String)],
    containers: &[(String, Vec<String>)],
) -> Vec<(String, Vec<String>)> {
    let mut groups = Vec::new();
    for (container, members) in containers {
        let live: Vec<String> = members
            .iter()
            .filter(|member| face_names.iter().any(|(_, name)| name == *member))
            .cloned()
            .collect();
        if !live.is_empty() {
            groups.push((container.clone(), live));
        }
    }
    groups
}

/// The CONTAINER-name spelling of each derived edge name.
///
/// Derived edge names embed face names (`{faceA}|{faceB}[n]`), so keying the caps
/// renamed every cap-adjacent edge along with them — a saved fillet on the top
/// edge of a box references a name no face pair spells any more. This recomputes
/// each edge's derived name with every member face name replaced by its container
/// name, and registers THAT as an alias for the edge.
///
/// It mirrors [`stamp_derived_edge_names`] step for step — same adjacency, same
/// encounter order, same per-base counter, substitution applied BEFORE the pair is
/// sorted (a substituted name can sort differently from the one it replaced). So
/// when the mapping is 1:1 — every model with a single-loop profile, i.e. every
/// model saved before per-loop naming — the alias is byte-identical to the name
/// that solid's edges used to carry.
fn collect_edge_group_aliases(
    solid: &BrepSolid,
    containers: &[(String, Vec<String>)],
) -> Vec<(String, Vec<String>)> {
    use std::collections::HashMap;
    if containers.is_empty() {
        return Vec::new();
    }
    // member face name -> its container name.
    let mut container_of: HashMap<&str, &str> = HashMap::new();
    for (container, members) in containers {
        for member in members {
            container_of.insert(member.as_str(), container.as_str());
        }
    }

    let (edge_faces, encounter_order) = collect_edge_face_names(solid);

    let solid_name_fallback = "Solid".to_string();
    let mut base_counts: HashMap<String, usize> = HashMap::new();
    let mut aliases: HashMap<String, Vec<String>> = HashMap::new();
    let mut order: Vec<String> = Vec::new();
    for edge_id in encounter_order {
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        // Only edges `stamp_derived_edge_names` actually named advance its
        // counter, so only those advance this one.
        if !edge.name.as_deref().is_none_or(|name| name.contains('|')) {
            continue;
        }
        let mut faces: Vec<String> = edge_faces
            .get(&edge_id)
            .map(|list| list.iter().filter(|name| !name.is_empty()).cloned().collect())
            .unwrap_or_default();
        // Substitute BEFORE sorting — a container name need not sort where the
        // member name it replaces did.
        let substituted = faces.iter().any(|name| container_of.contains_key(name.as_str()));
        for face in &mut faces {
            if let Some(container) = container_of.get(face.as_str()) {
                *face = (*container).to_string();
            }
        }
        faces.sort();
        let base = if faces.len() >= 2 {
            format!("{}|{}", faces[0], faces[1])
        } else {
            format!("{}|BOUNDARY", faces.first().unwrap_or(&solid_name_fallback))
        };
        let index = base_counts.entry(base.clone()).or_insert(0);
        let alias = format!("{base}[{index}]");
        *index += 1;
        // An edge touching no member face spells its own name — nothing to alias.
        // An edge whose alias IS its own name needs none either.
        let Some(live) = edge.name.clone() else { continue };
        if !substituted || live == alias {
            continue;
        }
        if aliases.entry(alias.clone()).or_default().is_empty() {
            order.push(alias.clone());
        }
        aliases.get_mut(&alias).expect("just inserted").push(live);
    }
    order
        .into_iter()
        .map(|alias| {
            let ids = aliases.remove(&alias).unwrap_or_default();
            (alias, ids)
        })
        .filter(|(_, ids)| !ids.is_empty())
        .collect()
}

/// Finalize several bodies using the optional boolean operation. Without an
/// operation or resolved targets, register each body separately. Otherwise fold
/// bodies into the targets in order and use the first target's name. Missing
/// targets are reported in `unresolved` for the caller to repair.
pub fn finalize_solids(ctx: &FeatureContext, bodies: Vec<(String, BrepSolid)>) -> FeatureResult {
    let boolean = read_boolean_param(ctx);
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    let separate = |mut result: FeatureResult, bodies: Vec<(String, BrepSolid)>| {
        for (name, solid) in bodies {
            result.added.push(register_added(solid, &name));
        }
        result
    };

    if boolean.operation == "NONE" || boolean.targets.is_empty() {
        return separate(result, bodies);
    }
    let operation = match parse_boolean_operation(&boolean.operation) {
        Ok(operation) => operation,
        Err(error) => return ctx.fail(error),
    };
    let resolved = resolve_solid_names(ctx.scene, &boolean.targets, &mut result.unresolved);
    if resolved.is_empty() {
        return separate(result, bodies);
    }
    let options = BooleanOptions {
        merge_coplanar_faces: boolean.merge_coplanar_faces,
        ..BooleanOptions::default()
    };
    let mut bodies = bodies.into_iter();
    let Some((_, first)) = bodies.next() else {
        return result; // no bodies at all (shouldn't happen; callers gate)
    };
    // Fold the first body through every target (mirrors `finalize_solid`), then
    // fold each remaining body into the running result (targets already merged in).
    let mut current = first;
    for (_, target_handle) in &resolved {
        let operand = crate::register_solid_value(current);
        let folded = crate::with_two_registered_solids(*target_handle, operand, |target, op_solid| {
            crate::boolean_operation(target, op_solid, operation, &options)
        });
        crate::free_registered_solid(operand);
        match folded {
            Ok(solid) => current = solid,
            Err(error) => return ctx.fail(format!("boolean {} failed: {error}", boolean.operation)),
        }
    }
    for (_, body) in bodies {
        match crate::boolean_operation(&current, &body, operation, &options) {
            Ok(solid) => current = solid,
            Err(error) => return ctx.fail(format!("boolean {} failed: {error}", boolean.operation)),
        }
    }
    result.added.push(register_added(current, &resolved[0].0));
    result.removed = resolved.into_iter().map(|(name, _)| name).collect();
    result
}

pub fn finalize_solid(ctx: &FeatureContext, base: BrepSolid, base_name: &str) -> FeatureResult {
    finalize_solid_grouped(ctx, base, base_name, &[])
}

/// [`finalize_solid`] carrying CONTAINER groups (the profile-swept features'
/// per-loop cap roll-ups) onto whichever solid ends up registered — the base
/// itself, or the boolean's result when one is folded. Members that the boolean
/// consumed drop out of their group on their own (`register_added_grouped`
/// resolves members against the FINAL solid).
pub fn finalize_solid_grouped(
    ctx: &FeatureContext,
    base: BrepSolid,
    base_name: &str,
    containers: &[(String, Vec<String>)],
) -> FeatureResult {
    let boolean = read_boolean_param(ctx);
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    if boolean.operation == "NONE" || boolean.targets.is_empty() {
        result.added.push(register_added_grouped(base, base_name, containers));
        return result;
    }

    let operation = match parse_boolean_operation(&boolean.operation) {
        Ok(operation) => operation,
        Err(error) => return ctx.fail(error),
    };

    let resolved = resolve_solid_names(ctx.scene, &boolean.targets, &mut result.unresolved);

    // Nothing resolved: emit the base un-booleaned; `unresolved` signals the caller to
    // repair + re-dispatch (NOT a hard error — Stage 5 gate).
    if resolved.is_empty() {
        result.added.push(register_added_grouped(base, base_name, containers));
        return result;
    }

    let options = BooleanOptions {
        merge_coplanar_faces: boolean.merge_coplanar_faces,
        ..BooleanOptions::default()
    };
    // Fold, keeping the running result as a LOCAL owned BrepSolid: register only
    // the operand needed for the two-solid borrow, fold, free that operand — no
    // leaked intermediates. The base primitive is never itself scene-resident.
    let mut current = base;
    for (_, target_handle) in &resolved {
        let operand_handle = crate::register_solid_value(current);
        let folded = crate::with_two_registered_solids(*target_handle, operand_handle, |target, operand| {
            crate::boolean_operation(target, operand, operation, &options)
        });
        crate::free_registered_solid(operand_handle);
        match folded {
            Ok(solid) => current = solid,
            Err(error) => return ctx.fail(format!("boolean {} failed: {error}", boolean.operation)),
        }
    }

    let result_name = resolved[0].0.clone();
    result
        .added
        .push(register_added_grouped(current, &result_name, containers));
    result.removed = resolved.into_iter().map(|(name, _)| name).collect();
    result
}

// ===========================================================================
// Dressup edge/face selection — shared by fillet.rs (F) and chamfer.rs (CH)
// ===========================================================================
//
// Fillet and chamfer share selection resolution; chamfer alone validates its
// `direction`.
// Each feature owns its numeric parameters, kernel call, and abort policy.

/// One solid and sampled edge points for a fillet or chamfer operation.
pub struct BlendTarget {
    pub handle: u32,
    pub name: String,
    pub edge_points: Vec<Vec3>,
    /// Canonical names parallel to `edge_points`, used to name generated faces.
    /// Unnamed edges fall back to `E{edge_id}`.
    pub edge_names: Vec<String>,
}

/// Resolved blend selection; callers decide how to handle cross-solid input.
pub struct BlendSelection {
    /// Present when at least one reference resolves, all to the same solid.
    pub target: Option<BlendTarget>,
    pub multi_solid: bool,
    /// Missing edge/face names for the caller to repair and redispatch.
    pub unresolved: Vec<String>,
}

/// One resolved ref: which topology entity of the owning solid it names.
enum ResolvedRef {
    Edge(u64),
    Face(u64),
}

/// A `reference_selection` entry -> its name: a plain string, or an object with a
/// `name` field. No token-splitting / `[N]`-index /
/// snapshot-scoring — exact match only (contract rule 1 forbids the heuristic).
pub(crate) fn reference_name(value: &serde_json::Value) -> Option<String> {
    let raw = match value {
        serde_json::Value::String(text) => Some(text.as_str()),
        serde_json::Value::Object(map) => map.get("name").and_then(|value| value.as_str()),
        _ => None,
    }?;
    let trimmed = raw.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_string())
}

/// The canonical scene name of an edge (`{faceA}|{faceB}[n]`, stamped by
/// `stamp_derived_edge_names`), used to name the blend face grown from it. An
/// unnamed / empty edge (e.g. a hand-built test solid never run through the
/// canonical pass) falls back to a deterministic `E{edge_id}` so parallel blend
/// faces still get DISTINCT, stable names.
fn edge_name_or_id(solid: &BrepSolid, edge_id: u64) -> String {
    solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .and_then(|edge| edge.name.clone())
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| format!("E{edge_id}"))
}

/// Midpoint of an edge's curve (`evaluate((t0 + t1) / 2)`) — the sample the kernel
/// decodes back to the edge.
fn sample_edge_midpoint(solid: &BrepSolid, edge_id: u64) -> Result<Vec3, String> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("dressup: edge {edge_id} not found on target solid"))?;
    let mid = (edge.t0 + edge.t1) * 0.5;
    edge.curve
        .evaluate(mid)
        .map_err(|error| format!("dressup: edge {edge_id} midpoint sample failed: {error}"))
}

/// Boundary edge ids of a face (its loops' coedges), deduped, skipping DEGENERATE
/// (pole) edges — "fillet a face" means "fillet all its real shared edges".
fn face_boundary_edges(solid: &BrepSolid, face_id: u64) -> Result<Vec<u64>, String> {
    let face = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| format!("dressup: face {face_id} not found on target solid"))?;
    let mut ids = Vec::new();
    let mut seen = FxHashSet::default();
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let degenerate = solid
                .edges
                .iter()
                .find(|edge| edge.id == coedge.edge_id)
                .map(|edge| edge.degenerate)
                .unwrap_or(false);
            if degenerate {
                continue;
            }
            if seen.insert(coedge.edge_id) {
                ids.push(coedge.edge_id);
            }
        }
    }
    Ok(ids)
}

/// Resolve the `edges` `reference_selection` (EDGE and/or FACE names) against the
/// live scene-map and sample one 3D point per selected edge. Exact-match only;
/// misses accumulate into `unresolved`. Errs only on a genuine geometry/lookup
/// failure while sampling the resolved solid.
pub fn resolve_blend_selection(ctx: &FeatureContext) -> Result<BlendSelection, String> {
    let names = reference_names(ctx.param("edges"));

    let mut unresolved = Vec::new();
    let mut resolved: Vec<(u32, ResolvedRef)> = Vec::new();
    for name in names {
        // Exact edge, exact face, then the CONTAINER groups — a name a model
        // stored before per-loop cap naming stands for every face (or edge) the
        // feature built from that profile, so a fillet on "the end cap" of a
        // 3-loop extrude blends all three, and one on a single-loop extrude
        // blends the one face it always meant.
        let edges = ctx.scene.resolve_edge_group(&name);
        let faces = ctx.scene.resolve_face_group(&name);
        if edges.is_empty() && faces.is_empty() {
            unresolved.push(name);
            continue;
        }
        for edge in edges {
            resolved.push((edge.handle, ResolvedRef::Edge(edge.edge_id)));
        }
        for face in faces {
            resolved.push((face.handle, ResolvedRef::Face(face.face_id)));
        }
    }

    // Distinct owning handles among the resolved refs.
    let mut handles: Vec<u32> = resolved.iter().map(|(handle, _)| *handle).collect();
    handles.sort_unstable();
    handles.dedup();
    if handles.len() > 1 {
        return Ok(BlendSelection {
            target: None,
            multi_solid: true,
            unresolved,
        });
    }
    let Some(&handle) = handles.first() else {
        // Nothing resolved: `unresolved` carries the misses (rule 1); no target.
        return Ok(BlendSelection {
            target: None,
            multi_solid: false,
            unresolved,
        });
    };

    // Reverse-lookup the target's scene name — the result reuses it (loop
    // removes-then-adds, freeing the consumed handle), matching the established dressup naming
    // (`new Solid(kernel, { name: targetSolid.name })`).
    let name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, &registered)| registered == handle)
        .map(|(name, _)| name.clone())
        .ok_or_else(|| format!("dressup: target handle {handle} has no scene name"))?;

    // Sample one point per selected edge; dedup edge ids GLOBALLY across all refs
    // (dedup spans the whole selection, not per-face) in a single short borrow.
    // The originating edge NAME is captured PARALLEL to each point so the grown
    // blend face can be named after the edge it came from.
    let (edge_points, edge_names) = crate::with_registered_solid_str(handle, |solid| {
        let mut seen = FxHashSet::default();
        let mut points = Vec::new();
        let mut names = Vec::new();
        for (_, entity) in &resolved {
            match entity {
                ResolvedRef::Edge(edge_id) => {
                    if seen.insert(*edge_id) {
                        points.push(sample_edge_midpoint(solid, *edge_id)?);
                        names.push(edge_name_or_id(solid, *edge_id));
                    }
                }
                ResolvedRef::Face(face_id) => {
                    for edge_id in face_boundary_edges(solid, *face_id)? {
                        if seen.insert(edge_id) {
                            points.push(sample_edge_midpoint(solid, edge_id)?);
                            names.push(edge_name_or_id(solid, edge_id));
                        }
                    }
                }
            }
        }
        Ok((points, names))
    })?;

    Ok(BlendSelection {
        target: Some(BlendTarget {
            handle,
            name,
            edge_points,
            edge_names,
        }),
        multi_solid: false,
        unresolved,
    })
}

/// Shared wall and corner prefix, `{featureID || 'F'}:BLEND`.
pub fn blend_face_base(id: &str) -> String {
    format!("{}:BLEND", if id.is_empty() { "F" } else { id })
}

/// Per-edge face name; distinct wall names keep derived blend-edge names stable.
pub fn blend_face_name(id: &str, edge_name: &str) -> String {
    format!("{}:{}", blend_face_base(id), edge_name)
}

/// Reject chamfer directions that depended on the removed mesh blend pipeline.
/// Fillet has no `direction` param: its blend side is always classified per edge.
pub fn require_blend_direction(ctx: &FeatureContext, operation: &str) -> Result<(), String> {
    let direction = ctx
        .param("direction")
        .and_then(|value| value.as_str())
        .map(|text| text.trim().to_uppercase())
        .filter(|text| !text.is_empty())
        .unwrap_or_else(|| "AUTO".to_string());
    if direction != "AUTO" && direction != "INSET" {
        return Err(format!(
            "{operation} direction '{direction}' belonged to the removed legacy mesh pipeline (only AUTO/INSET exist)"
        ));
    }
    Ok(())
}

/// Read a finite parameter value, falling back for missing/null entries or
/// evaluation errors. Used by features whose optional fields are lenient.
pub(super) fn number_or_default(ctx: &FeatureContext, key: &str, default: f64) -> f64 {
    match ctx.param(key) {
        None | Some(serde_json::Value::Null) => default,
        Some(_) => match ctx.number(key) {
            Ok(value) if value.is_finite() => value,
            _ => default,
        },
    }
}

/// An OPTIONAL numeric param: `None` when absent, else evaluated via
/// [`FeatureContext::number`] (a hard error on a broken expression, like the
/// primitive features). Used for the dressup gate/shape params (`inflate`,
/// `nudgeFaceDistance`, `radiusEnd`, `distance2`, `angle`).
pub fn optional_number(ctx: &FeatureContext, key: &str) -> Result<Option<f64>, String> {
    if ctx.param(key).is_some() {
        ctx.number(key).map(Some)
    } else {
        Ok(None)
    }
}

// ===========================================================================
// The ONE feature-transform convention (XFORM, ACOMP, gizmo write-back)
// ===========================================================================

/// Compose the row-major TRS matrix `[ RS | translate + pivot − RS·pivot ]`
/// with `R` the intrinsic `XYZ` Euler rotation (= `Rx·Ry·Rz`) and `S =
/// diag(scale)` — the SINGLE feature-transform compose. XFORM, the ACOMP
/// instance pose, and any solver/gizmo pose write-back must all speak this
/// convention; forking it would make the same stored angles place geometry
/// differently per consumer.
pub fn compose_trs_matrix(
    translate: [f64; 3],
    rotate_rad: [f64; 3],
    scale: [f64; 3],
    pivot: [f64; 3],
) -> [f64; 16] {
    let (a, b) = (rotate_rad[0].cos(), rotate_rad[0].sin());
    let (c, d) = (rotate_rad[1].cos(), rotate_rad[1].sin());
    let (e, f) = (rotate_rad[2].cos(), rotate_rad[2].sin());
    let (ae, af, be, bf) = (a * e, a * f, b * e, b * f);
    // Intrinsic 'XYZ' Euler rotation (row-major) = Rx·Ry·Rz.
    let r = [
        [c * e, -c * f, d],
        [af + be * d, ae - bf * d, -b * c],
        [bf - ae * d, be + af * d, a * c],
    ];
    // RS = R·diag(scale): scale each column (matches the standard TRS compose).
    let mut rs = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            rs[i][j] = r[i][j] * scale[j];
        }
    }
    let rs_pivot = [
        rs[0][0] * pivot[0] + rs[0][1] * pivot[1] + rs[0][2] * pivot[2],
        rs[1][0] * pivot[0] + rs[1][1] * pivot[1] + rs[1][2] * pivot[2],
        rs[2][0] * pivot[0] + rs[2][1] * pivot[1] + rs[2][2] * pivot[2],
    ];
    let tx = translate[0] + pivot[0] - rs_pivot[0];
    let ty = translate[1] + pivot[1] - rs_pivot[1];
    let tz = translate[2] + pivot[2] - rs_pivot[2];
    [
        rs[0][0], rs[0][1], rs[0][2], tx, //
        rs[1][0], rs[1][1], rs[1][2], ty, //
        rs[2][0], rs[2][1], rs[2][2], tz, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

/// Read a vec3 VALUE (array `[x,y,z]` or object `{x,y,z}`); each component may
/// be a JSON number, an expression string (evaluated against the shared env),
/// or null/missing (→ default). `None`/`Null` as the whole value is the
/// default. Anything else is a loud error naming `key`. The shared reader for
/// TOP-LEVEL vec3 params (via [`FeatureContext::param`]) and NESTED ones (the
/// ACOMP `transform.translate` / `transform.rotateEulerDeg`).
pub fn vec3_from_value(
    env: &crate::feature_pipeline::Env,
    value: Option<&serde_json::Value>,
    key: &str,
    default: [f64; 3],
) -> Result<[f64; 3], String> {
    let Some(value) = value.filter(|value| !value.is_null()) else {
        return Ok(default);
    };
    let component = |slot: Option<&serde_json::Value>, fallback: f64| -> Result<f64, String> {
        match slot {
            None | Some(serde_json::Value::Null) => Ok(fallback),
            Some(serde_json::Value::Number(number)) => number
                .as_f64()
                .ok_or_else(|| format!("param `{key}` has a non-finite component")),
            Some(serde_json::Value::String(source)) => {
                let number = env
                    .eval(source)
                    .map_err(|error| format!("param `{key}`: {error}"))?;
                // An expression may evaluate to a NON-finite number (`1/0`) — that
                // must fail HERE, naming the slot, rather than travel on as a
                // point coordinate or reach `AffineTransform::new` as a nameless
                // "matrix must be finite".
                if !number.is_finite() {
                    return Err(format!(
                        "param `{key}`: `{source}` evaluated to {number}"
                    ));
                }
                Ok(number)
            }
            Some(other) => Err(format!(
                "param `{key}` component must be a number or expression, found {other}"
            )),
        }
    };
    if let Some(array) = value.as_array() {
        return Ok([
            component(array.first(), default[0])?,
            component(array.get(1), default[1])?,
            component(array.get(2), default[2])?,
        ]);
    }
    if value.is_object() {
        return Ok([
            component(value.get("x"), default[0])?,
            component(value.get("y"), default[1])?,
            component(value.get("z"), default[2])?,
        ]);
    }
    Err(format!("param `{key}` must be a vec3 array or object"))
}

// ===========================================================================
// Reference-name + geometric-input resolution shared by profile-consumers
// ===========================================================================

/// Resolve exact names in selection order, retaining duplicates and appending misses.
pub(super) fn resolve_solid_names(
    scene: &crate::feature_pipeline::SceneMap,
    names: &[String],
    unresolved: &mut Vec<String>,
) -> Vec<(String, u32)> {
    let mut resolved = Vec::new();
    for name in names {
        match scene.resolve_solid(name) {
            Some(handle) => resolved.push((name.clone(), handle)),
            None => unresolved.push(name.clone()),
        }
    }
    resolved
}

/// Use an explicit sheet selection, or the sole sheet-metal body in the scene.
/// Missing or ambiguous implicit targets remain unselected.
pub(super) fn sheet_body_name(ctx: &FeatureContext) -> Option<String> {
    first_reference_name(ctx.param("sheet")).or_else(|| {
        let mut bodies = ctx.scene.solids.iter().filter_map(|(name, &handle)| {
            crate::feature_pipeline::sheet_metal::get_tree(handle).map(|_| name.clone())
        });
        match (bodies.next(), bodies.next()) {
            (Some(only), None) => Some(only),
            _ => None,
        }
    })
}

/// Read a single selection without descending into nested arrays.
/// An empty first textual entry means no selection; later entries are ignored.
pub(super) fn single_reference_name(value: Option<&serde_json::Value>) -> Option<String> {
    let name = match value {
        Some(serde_json::Value::String(text)) => Some(text.clone()),
        Some(serde_json::Value::Array(array)) => array.iter().find_map(|entry| {
            entry
                .as_str()
                .or_else(|| entry.get("name").and_then(|name| name.as_str()))
                .map(str::to_string)
        }),
        Some(object @ serde_json::Value::Object(_)) => {
            object.get("name").and_then(|name| name.as_str()).map(str::to_string)
        }
        _ => None,
    }?;
    let trimmed = name.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_string())
}

/// The first reference name of a `reference_selection` param value (a plain
/// string, a `{name}` object, or an array of either — post-sanitization it is an
/// array). Exact match only; no `[N]` index / snapshot scoring.
pub fn first_reference_name(value: Option<&serde_json::Value>) -> Option<String> {
    fn one(value: &serde_json::Value) -> Option<String> {
        match value {
            serde_json::Value::Array(items) => items.iter().find_map(one),
            other => reference_name(other),
        }
    }
    value.and_then(one)
}

/// Normalize a PROFILE reference name: a committed sketch is shown as a render-side
/// display SHEET whose planar face is PICKED as `{sketch}:FACE`, but the kernel run
/// never materializes that sheet — the pick actually names the sketch's PROFILE. So
/// strip a trailing `:FACE` to the sketch base, and every profile-consumer's resolve
/// + cap-naming + sketch-consume then uses the canonical id. Resident SOLID faces are
/// `{solid}|{face}[n]` and never end in `:FACE`, so real faces pass through untouched.
/// Apply ONLY to `profile` fields (never to `path`/`axis`).
pub fn normalize_profile_alias(name: String) -> String {
    match name.strip_suffix(":FACE") {
        Some(base) => base.to_string(),
        None => name,
    }
}

/// Names from an array-valued reference selection, preserving order and duplicates.
/// Missing and non-array values are ignored; entries use [`reference_name`].
pub(crate) fn reference_name_array(value: Option<&serde_json::Value>) -> Vec<String> {
    value
        .and_then(serde_json::Value::as_array)
        .map(|items| items.iter().filter_map(reference_name).collect())
        .unwrap_or_default()
}

/// ALL reference names of a `reference_selection` param, in order (a `multiple`
/// selection). Also accepts a single string or `{name}` object; empties are dropped.
pub fn reference_names(value: Option<&serde_json::Value>) -> Vec<String> {
    match value {
        Some(serde_json::Value::Array(items)) => items.iter().filter_map(reference_name).collect(),
        Some(other) => reference_name(other).into_iter().collect(),
        None => Vec::new(),
    }
}

/// Reference names with duplicates removed, retaining first-occurrence order.
pub(super) fn unique_reference_names(value: Option<&serde_json::Value>) -> Vec<String> {
    let mut seen = std::collections::HashSet::new();
    reference_names(value)
        .into_iter()
        .filter(|name| seen.insert(name.clone()))
        .collect()
}

/// `consumeProfileSketch` (default true) — the profile-consumers remove the sketch
/// after building, echoing its name into `removed` for the caller's display cleanup.
pub fn consume_profile_sketch(ctx: &FeatureContext) -> bool {
    !matches!(
        ctx.param("consumeProfileSketch"),
        Some(serde_json::Value::Bool(false))
    )
}

/// Echo the consumed sketch's base name (`{ref}` with any `:PROFILE` stripped) into
/// `result.removed` when `consumeProfileSketch` is set. Sketches register no scene
/// solid, so this only signals caller-side display cleanup.
pub fn consume_sketch(ctx: &FeatureContext, reference_name: &str, result: &mut FeatureResult) {
    if !consume_profile_sketch(ctx) {
        return;
    }
    consume_sketch_always(reference_name, result);
}

/// Sheet-metal contract: a sketch that fed a generated piece of geometry is
/// ALWAYS consumed (removed from the scene) — there is no keep-the-sketch
/// switch. Non-sheet-metal profile consumers keep the `consumeProfileSketch`
/// gate via `consume_sketch` above.
pub fn consume_sketch_always(reference_name: &str, result: &mut FeatureResult) {
    let base = reference_name
        .strip_suffix(":PROFILE")
        .unwrap_or(reference_name)
        .to_string();
    if !result.removed.contains(&base) {
        result.removed.push(base);
    }
}

/// Map one profile loop's WORLD-space curves into the profile's own plane as
/// local `(u, v, 0)` curves: each control point `p` → `((p−origin)·x, (p−origin)·y, 0)`,
/// weights kept. A rigid in-plane change of frame, so rational curves (circles /
/// arcs) stay EXACT — the sheet-metal `Flat::holes` contract (hole loops in
/// flat-local coordinates at `z = 0`).
pub fn profile_loop_uv_curves(
    profile: &crate::feature_pipeline::SketchProfile,
    profile_loop: &crate::feature_pipeline::ProfileLoop,
) -> Result<Vec<NurbsCurve>, String> {
    let mut curves = Vec::with_capacity(profile_loop.curves.len());
    for curve in &profile_loop.curves {
        let mut control_points = Vec::with_capacity(curve.control_points.len());
        for cp in &curve.control_points {
            let delta = cp.point()?.sub(profile.origin);
            control_points.push(crate::Vec4::from_point(
                Vec3::new(delta.dot(profile.x_axis), delta.dot(profile.y_axis), 0.0),
                cp.w,
            ));
        }
        curves.push(NurbsCurve::new(
            curve.degree,
            curve.knots.clone(),
            control_points,
        )?);
    }
    Ok(curves)
}

/// Resolve a sweep/rib PATH reference to its ordered world curve chain: a SKETCH
/// path (`scene.resolve_path`), a single resident solid EDGE (its curve TRIMMED
/// to the edge — [`edge_curve`]), or a sketch GEOMETRY name `{sketchId}:G{gid}`
/// → the owning sketch's published chain (the tube.rs convention —
/// per-geometry curves are not scene-indexed, and a chain holding more than the
/// named segment defers loudly downstream). A consumer wanting one path curve
/// uses the chain when it is length 1.
pub fn resolve_path(ctx: &FeatureContext, name: &str) -> Result<Vec<NurbsCurve>, String> {
    if let Some(chain) = ctx.scene.resolve_path(name) {
        return Ok(chain.clone());
    }
    if let Some(edge) = ctx.scene.resolve_edge(name) {
        return Ok(vec![edge_curve(edge)?]);
    }
    if let Some((owner, geometry)) = name.rsplit_once(':') {
        if geometry.len() > 1 && geometry.starts_with('G') {
            if let Some(chain) = ctx.scene.resolve_path(owner) {
                return Ok(chain.clone());
            }
        }
    }
    Err(format!(
        "path '{name}' not found (no sketch path or resident edge)"
    ))
}

/// ONE segment of a resolved sweep path: the world curve, plus the SOURCE NAME
/// it came from (`{sketchId}:G{gid}` for a sketch geometry, the edge's own scene
/// name for a resident solid edge).
///
/// The name is the whole point: a feature that builds one PORTION per path
/// segment names that portion's faces after the segment, so adding, removing or
/// re-picking a path edge never renames a face built from a segment that did not
/// change. Naming by chain POSITION would renumber every downstream face the
/// moment a segment is inserted.
#[derive(Debug, Clone)]
pub struct PathSegment {
    pub name: String,
    pub curve: NurbsCurve,
    /// The segment's SCREW AXIS direction when its publisher knows it is a helix
    /// ([`SceneMap::resolve_path_segment_axes`]). Reversing the segment leaves it
    /// valid: a frame referenced to an axis does not depend on the axis' sign.
    pub axis: Option<Vec3>,
}

/// Resolve a `path` `reference_selection` — one name or MANY — to a single
/// ordered, head-to-tail connected chain of NAMED segments.
///
/// Each reference contributes its curves through [`resolve_path`] (a whole
/// SKETCH's published chain, a single `{sketchId}:G{gid}` sketch segment, or a
/// resident solid EDGE), and each curve carries a name: the publisher's
/// per-segment names when it published them
/// ([`SceneMap::resolve_path_segment_names`]), else the reference name itself
/// (a one-curve reference) or `{reference}[i]` (an unnamed multi-curve chain).
/// A curve already contributed under the same name is dropped, so selecting a
/// sketch AND one of its own segments is not a duplicate.
///
/// The segments are then CHAINED by endpoint coincidence: the first reference
/// seeds the chain in its natural orientation, the chain grows from its tail and
/// then from its head (mirroring the sketch chainer), and a segment that has to
/// run backwards to join is reversed. So the picking ORDER does not matter —
/// the same set of edges always yields the same chain. Any segment that reaches
/// neither end is a disconnected or branching selection, and is reported BY NAME
/// rather than silently dropped.
/// Resolve a `path` `reference_selection` to the sweep's own PATH TYPE: the
/// ordered chain [`resolve_path_chain`] produces, CLASSIFIED once
/// ([`crate::SweepPath`]) — open or closed, the plane it lies in if it lies in
/// one, and the continuity of every joint including a closed path's closing one.
///
/// Every sweep refusal that used to answer one of those questions for itself now
/// reads this: the builder's corner gate, the closed-path lane, and `SW`'s own
/// `translate` fold-back rule, which could previously only report a closed path
/// as the consequence it happened to notice (a segment that must reverse).
/// Resolution, order and orientation are unchanged — this classifies the chain
/// the resolver produced, it does not re-resolve or re-order it.
pub fn resolve_sweep_path(
    ctx: &FeatureContext,
    names: &[String],
) -> Result<crate::SweepPath, String> {
    let segments = resolve_path_chain(ctx, names)?;
    let mut curves = Vec::with_capacity(segments.len());
    let mut segment_names = Vec::with_capacity(segments.len());
    let mut axes = Vec::with_capacity(segments.len());
    for segment in segments {
        curves.push(segment.curve);
        segment_names.push(segment.name);
        axes.push(segment.axis);
    }
    crate::SweepPath::new(curves, segment_names)?
        .with_screw_axes(axes)
        .map_err(String::from)
}

pub fn resolve_path_chain(
    ctx: &FeatureContext,
    names: &[String],
) -> Result<Vec<PathSegment>, String> {
    let mut segments: Vec<PathSegment> = Vec::new();
    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    for reference in names {
        let curves = resolve_path(ctx, reference)?;
        let published = ctx.scene.resolve_path_segment_names(reference);
        let published_axes = ctx.scene.resolve_path_segment_axes(reference);
        let single = curves.len() == 1;
        for (index, curve) in curves.into_iter().enumerate() {
            let name = published
                .and_then(|names| names.get(index).cloned().flatten())
                .filter(|name| !name.trim().is_empty())
                .unwrap_or_else(|| {
                    if single {
                        reference.clone()
                    } else {
                        format!("{reference}[{index}]")
                    }
                });
            let axis = published_axes
                .and_then(|axes| axes.get(index).copied().flatten())
                .map(|axis| axis.direction);
            if seen.insert(name.clone()) {
                segments.push(PathSegment { name, curve, axis });
            }
        }
    }
    if segments.is_empty() {
        return Err("path selection resolved to no curves".into());
    }
    chain_path_segments(segments)
}

/// A path curve's world endpoints (`(start, end)` at its domain ends).
fn path_endpoints(curve: &NurbsCurve) -> Result<(Vec3, Vec3), String> {
    let [t0, t1] = curve.domain()?;
    Ok((curve.evaluate(t0)?, curve.evaluate(t1)?))
}

/// Order `segments` head-to-tail (see [`resolve_path_chain`]). Segments that join
/// nothing are reported by name.
fn chain_path_segments(segments: Vec<PathSegment>) -> Result<Vec<PathSegment>, String> {
    if segments.len() < 2 {
        return Ok(segments);
    }
    let ends: Vec<(Vec3, Vec3)> = segments
        .iter()
        .map(|segment| path_endpoints(&segment.curve))
        .collect::<Result<_, _>>()?;
    // Scale-relative join tolerance, on the same footing as the sketch chainer's
    // absolute 1e-5: individually picked solid edges meet only to within the
    // tolerance their own builders left behind.
    let scale = ends
        .iter()
        .flat_map(|(a, b)| [a, b])
        .map(|point| point.sub(ends[0].0).length())
        .fold(1.0_f64, f64::max);
    let tolerance = 1e-5 * scale;
    let joins = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;

    let mut used = vec![false; segments.len()];
    used[0] = true;
    // `(index, reversed)` in head-to-tail order, seeded on the FIRST picked
    // segment in its natural orientation.
    let mut order: Vec<(usize, bool)> = vec![(0, false)];
    let mut head = ends[0].0;
    let mut tail = ends[0].1;
    loop {
        // Closed chain: the tail returned to the head — stop before overshooting
        // into a segment attached at the seam (the sketch chainer's rule).
        if order.len() > 1 && joins(tail, head) {
            break;
        }
        if let Some((index, reversed, far)) = attach_path_segment(&ends, &used, tail, tolerance) {
            used[index] = true;
            order.push((index, reversed));
            tail = far;
            continue;
        }
        if let Some((index, reversed, far)) = attach_path_segment(&ends, &used, head, tolerance) {
            used[index] = true;
            // Walking head-ward: `attach_path_segment` orients tail-ward, so flip.
            order.insert(0, (index, !reversed));
            head = far;
            continue;
        }
        break;
    }
    let stranded: Vec<&str> = segments
        .iter()
        .zip(&used)
        .filter(|(_, used)| !**used)
        .map(|(segment, _)| segment.name.as_str())
        .collect();
    if !stranded.is_empty() {
        return Err(format!(
            "the selected path edges do not form ONE connected chain — {} joins neither \
             end of the chain the other selections form (a sweep path must be a single \
             head-to-tail run; select connected edges, or one branch at a time)",
            stranded.join(", ")
        ));
    }
    let mut chained = Vec::with_capacity(segments.len());
    let mut taken: Vec<Option<PathSegment>> = segments.into_iter().map(Some).collect();
    for (index, reversed) in order {
        let mut segment = taken[index].take().expect("each segment placed once");
        if reversed {
            segment.curve = segment.curve.reversed()?;
        }
        chained.push(segment);
    }
    Ok(chained)
}

/// The first unused segment with an endpoint at `cursor`, oriented to walk AWAY
/// from it: `(index, reversed, far_end)`. Index-order scan, deterministic.
fn attach_path_segment(
    ends: &[(Vec3, Vec3)],
    used: &[bool],
    cursor: Vec3,
    tolerance: f64,
) -> Option<(usize, bool, Vec3)> {
    for (index, (start, end)) in ends.iter().enumerate() {
        if used[index] {
            continue;
        }
        if start.sub(cursor).length() <= tolerance {
            return Some((index, false, *end));
        }
        if end.sub(cursor).length() <= tolerance {
            return Some((index, true, *start));
        }
    }
    None
}

/// Translate every curve of a profile loop by `offset` — the per-portion
/// placement a multi-segment sweep needs (each path segment sweeps a COPY of the
/// profile, moved to that segment's start).
pub fn translate_curves(curves: &[NurbsCurve], offset: Vec3) -> Result<Vec<NurbsCurve>, String> {
    curves
        .iter()
        .map(|curve| translate_curve(curve, offset))
        .collect()
}

/// The SKETCH PLANE governing a path/profile reference, if it has one: the
/// reference name itself (a SKETCH publishes its resolved frame under its own id)
/// or the owner prefix of a sketch-geometry name (`{sketchId}:G{gid}`,
/// `{sketchId}:PROFILE`, `{sketchId}:FACE`). `None` for a non-sketch reference —
/// a resident solid EDGE publishes no frame.
///
/// This is the plane a sketch was AUTHORED on, and it is the answer to every
/// "which way is out of this chain?" question a consumer would otherwise re-derive
/// from the chain's own bends: the re-derivation is sign-ambiguous (it follows the
/// order the segments happen to be drawn in), and for a chain with no bend at all —
/// a single straight segment — it does not exist. Consumed by the sheet-metal
/// contour flange, and by the rib.
pub fn sketch_plane_frame(ctx: &FeatureContext, names: &[String]) -> Option<Frame> {
    for name in names {
        if let Some(frame) = ctx.scene.resolve_frame(name) {
            return Some(frame);
        }
        if let Some((owner, _)) = name.rsplit_once(':') {
            if let Some(frame) = ctx.scene.resolve_frame(owner) {
                return Some(frame);
            }
        }
    }
    None
}

/// The base name a sketch-consuming feature should REMOVE for a reference: the
/// owning SKETCH for a sketch-geometry pick (`{sketchId}:G{gid}` whose owner
/// published a frame), else the raw reference name (`consume_sketch_always` strips
/// `:PROFILE` itself; non-sketch names are harmless display no-ops).
///
/// A viewport pick on a sketch's drawn SEGMENT stores `{sketchId}:G{gid}`, so
/// without this the consume would echo a name no display knows and the consumed
/// sketch would linger in the scene. Consumed by the sheet-metal contour flange,
/// and by the rib.
pub fn sketch_base_name(ctx: &FeatureContext, name: &str) -> String {
    if let Some((owner, tail)) = name.rsplit_once(':') {
        if tail.len() > 1 && tail.starts_with('G') && ctx.scene.resolve_frame(owner).is_some() {
            return owner.to_string();
        }
    }
    name.to_string()
}

/// A curve trimmed to the parameter range `[t0, t1]` (the `coalesce.rs` pattern):
/// skip a split whose parameter sits within [`crate::NurbsCurve::split`]'s absolute
/// knot tolerance of a domain end, or the guard inside `split` rejects it. The
/// result keeps the original parameterization, so its domain IS `[t0, t1]`.
pub fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    let [start, end] = curve.domain()?;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if t0 > start + epsilon && t0 < end - epsilon {
        result = result.split(t0)?.1;
    }
    let domain = result.domain()?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        result = result.split(t1)?.0;
    }
    Ok(result)
}

/// The world curve of one resident solid EDGE, TRIMMED to that edge's own
/// `[t0, t1]`.
///
/// An `EdgeRecord` carries a curve plus the subrange of it the edge actually
/// spans, and the two disagree constantly: every boolean and every dressup that
/// shortens an edge rewrites `t0`/`t1` and leaves the whole underlying curve in
/// place (a box edge whose end is eaten by a fillet keeps its full 20mm line and
/// records `t=[0, 0.8]`). A consumer that clones `record.curve` and reads its
/// DOMAIN therefore builds along a curve that runs past the edge the user picked
/// — a tube or a sweep that overshoots into thin air. Everything that turns a
/// picked edge into a path goes through here so that cannot happen once per
/// feature.
pub fn edge_curve(edge: EdgeRef) -> Result<NurbsCurve, String> {
    crate::with_registered_solid_str(edge.handle, |solid| {
        let record = solid
            .edges
            .iter()
            .find(|candidate| candidate.id == edge.edge_id)
            .ok_or_else(|| format!("edge {} not found on solid", edge.edge_id))?;
        trimmed_curve(&record.curve, record.t0, record.t1)
    })
}

/// Resolve a revolve/sweep axis from a resident solid EDGE: the axis passes through
/// the edge's start and runs along its chord (start → end). A straight edge is
/// exact; a curved edge yields its chord (best-effort, matching the established edge-axis use).
pub fn edge_axis(edge: EdgeRef) -> Result<Axis, String> {
    crate::with_registered_solid_str(edge.handle, |solid| {
        let record = solid
            .edges
            .iter()
            .find(|candidate| candidate.id == edge.edge_id)
            .ok_or_else(|| format!("edge {} not found on solid", edge.edge_id))?;
        let start = record.curve.evaluate(record.t0)?;
        let end = record.curve.evaluate(record.t1)?;
        let direction = end.sub(start).normalized()?;
        Ok(Axis {
            point: start,
            direction,
        })
    })
}

// ===========================================================================
// Hole-loop subtraction (multi-loop regions) shared by the profile-consumers
// ===========================================================================
//
// A [`SketchProfile`] region carries `region[0]` = outer boundary,
// `region[1..]` = its containment-classified holes. Every profile-consumer
// (extrude, sweep, revolve, path sweep, loft) builds each region's OUTER solid
// with its own sweep, cuts the region's holes with the SAME sweep via
// [`subtract_region_holes`], then unions the region solids
// ([`union_region_solids`]). The nesting FOREST machinery below stays general
// (a hole group may carry children), though the sketch classifier promotes an
// island inside a hole to its own region, so sketch-produced regions are flat.

/// Rigid-translate a curve by `offset` (homogeneous-safe: rational weights kept).
pub fn translate_curve(curve: &NurbsCurve, offset: Vec3) -> Result<NurbsCurve, String> {
    let mut control_points = Vec::with_capacity(curve.control_points.len());
    for cp in &curve.control_points {
        let cartesian = cp.point()?;
        control_points.push(crate::Vec4::from_point(cartesian.add(offset), cp.w));
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
}

/// One inner loop of a region with its DIRECTLY nested loops: `region[index]`
/// cuts material at even-odd `depth` 0; each child is one level deeper.
pub struct HoleGroup {
    /// Index into the region's loops (>= 1).
    pub index: usize,
    /// Nesting depth below the outer loop (0 = a direct hole).
    pub depth: usize,
    pub children: Vec<HoleGroup>,
}

/// Classify `region[1..]` into a containment FOREST. Loops are sampled into 2D
/// polygons in the profile's own plane frame; loop `i` nests inside loop `j`
/// iff a representative point of `i` lies in `j`'s polygon (valid profiles
/// never intersect, so one point decides). Returns the depth-0 groups in loop
/// order.
pub fn hole_nesting(
    profile: &SketchProfile,
    region: &[crate::feature_pipeline::ProfileLoop],
) -> Result<Vec<HoleGroup>, String> {
    let count = region.len().saturating_sub(1);
    if count == 0 {
        return Ok(Vec::new());
    }
    // Sampled UV polygon per inner loop (8 samples per curve).
    let mut polygons: Vec<Vec<[f64; 2]>> = Vec::with_capacity(count);
    for hole in &region[1..] {
        let mut polygon = Vec::new();
        for curve in &hole.curves {
            let [t0, t1] = curve.domain()?;
            for step in 0..8 {
                let t = t0 + (t1 - t0) * (step as f64 / 8.0);
                let point = curve.evaluate(t)?;
                let delta = point.sub(profile.origin);
                polygon.push([delta.dot(profile.x_axis), delta.dot(profile.y_axis)]);
            }
        }
        polygons.push(polygon);
    }
    // Pairwise containment + per-loop depth (# of loops containing it).
    let mut contains = vec![vec![false; count]; count];
    for i in 0..count {
        let Some(representative) = polygons[i].first().copied() else {
            continue;
        };
        for j in 0..count {
            if i != j && point_in_polygon_uv(representative, &polygons[j]) {
                contains[i][j] = true;
            }
        }
    }
    let depths: Vec<usize> = (0..count)
        .map(|i| contains[i].iter().filter(|inside| **inside).count())
        .collect();
    Ok((0..count)
        .filter(|&i| depths[i] == 0)
        .map(|i| hole_group(i, &contains, &depths))
        .collect())
}

/// Build the [`HoleGroup`] rooted at inner-loop `i`: its children are the loops
/// directly inside it (contained by it, exactly one level deeper).
fn hole_group(i: usize, contains: &[Vec<bool>], depths: &[usize]) -> HoleGroup {
    let children = (0..depths.len())
        .filter(|&k| contains[k][i] && depths[k] == depths[i] + 1)
        .map(|k| hole_group(k, contains, depths))
        .collect();
    HoleGroup {
        index: i + 1,
        depth: depths[i],
        children,
    }
}

/// Standard even-odd ray-cast point-in-polygon in the profile's UV plane.
fn point_in_polygon_uv(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let count = polygon.len();
    if count < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = count - 1;
    for i in 0..count {
        let pi = polygon[i];
        let pj = polygon[j];
        let intersects = (pi[1] > point[1]) != (pj[1] > point[1])
            && point[0] < (pj[0] - pi[0]) * (point[1] - pi[1]) / (pj[1] - pi[1]) + pi[0];
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Stamp `{id}:HOLE:{key}` on every still-unnamed face of a hole cutter — the
/// extrude hole-cutter convention every profile-consumer shares (the boolean
/// propagates these names onto the cut walls). `key` is the hole loop's STABLE
/// identity ([`ProfileLoop::key`]), not its position, so adding or removing one
/// hole never renames another's walls. `segment` keys the name further for a
/// feature that cuts the SAME hole once per path segment (see [`hole_face_name`]).
pub fn name_hole_cutter_faces(cutter: &mut BrepSolid, id: &str, key: &str, segment: Option<&str>) {
    let name = hole_face_name(id, key, segment);
    for face in cutter
        .shells
        .iter_mut()
        .flat_map(|shell| shell.faces.iter_mut())
    {
        if face.name.is_none() {
            face.name = Some(name.clone());
        }
    }
}

/// A hole wall's face name: `{id}:HOLE:{key}`, and `{id}:HOLE:{key}:{segment}`
/// when the feature cuts the same hole once PER PATH SEGMENT (the multi-segment
/// sweep). The un-keyed spelling is then the CONTAINER standing for all of them,
/// exactly as it is for the per-loop caps.
pub fn hole_face_name(id: &str, key: &str, segment: Option<&str>) -> String {
    match segment {
        Some(segment) => format!("{id}:HOLE:{key}:{segment}"),
        None => format!("{id}:HOLE:{key}"),
    }
}

/// Boolean-subtract `cutter` from `body` with `merge_coplanar_faces` — the
/// hole-cut boolean (register both, subtract, free both).
pub fn subtract_solid(body: BrepSolid, cutter: BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let options = BooleanOptions {
        merge_coplanar_faces: true,
        ..BooleanOptions::default()
    };
    let body_handle = crate::register_solid_value(body);
    let cutter_handle = crate::register_solid_value(cutter);
    let cut = crate::with_two_registered_solids(body_handle, cutter_handle, |body, tool| {
        crate::boolean_operation(body, tool, BooleanOperation::Subtract, &options)
    });
    crate::free_registered_solid(body_handle);
    crate::free_registered_solid(cutter_handle);
    cut
}

/// The straight-sweep hole cutter: the hole loop extruded into a prism that
/// clears BOTH caps (offset `margin` before the start plane, `2*margin`
/// over-long — the sheet-metal cutout pattern). `depth` scales the margin so a
/// nested island's prism strictly overhangs its parent hole's prism caps (no
/// coincident cutter-on-cutter cap faces); depth 0 is extrude's exact margin.
pub fn hole_prism(
    hole_curves: &[NurbsCurve],
    direction: Vec3,
    distance: f64,
    depth: usize,
) -> Result<BrepSolid, KernelRefusal> {
    let dir = direction.normalized().or_input(KernelStage::Collect, "direction")?;
    let span = distance.abs();
    let margin = span.max(1.0) * (depth as f64 + 1.0);
    let start_t = distance.min(0.0) - margin;
    let length = span + 2.0 * margin;
    let offset = dir.scale(start_t);
    let mut curves = Vec::with_capacity(hole_curves.len());
    for curve in hole_curves {
        curves.push(translate_curve(curve, offset).or_refuse(KernelStage::Collect, "translate_curve")?);
    }
    crate::extrude_profile_brep(&curves, dir, length)
}

/// Subtract one hole loop from `body` as a through prism — extrude's
/// `subtract_hole`, verbatim (build the over-long prism, stamp
/// `{id}:HOLE:{index}` on its faces, boolean-subtract with coplanar merge). A
/// degenerate (< 2 curve) hole loop is nothing to cut.
pub fn subtract_hole_prism(
    feature: &str,
    body: BrepSolid,
    hole_curves: &[NurbsCurve],
    direction: Vec3,
    distance: f64,
    id: &str,
    key: &str,
) -> Result<BrepSolid, String> {
    if hole_curves.len() < 2 {
        return Ok(body);
    }
    let mut cutter = hole_prism(hole_curves, direction, distance, 0)?;
    name_hole_cutter_faces(&mut cutter, id, key, None);
    subtract_solid(body, cutter)
        .map_err(|error| format!("{feature}: hole {key} cut failed: {error}"))
}

/// Subtract every inner loop of `region` from `body`, nesting-aware: each
/// depth-0 hole becomes ONE cutter — its own swept solid minus its children's
/// sub-cutters (recursively). `build` sweeps `region[loop_index]` with the
/// FEATURE's own sweep (prism / revolve / path / loft); it receives the loop's
/// nesting depth for margin scaling. Cutter faces are stamped `{id}:HOLE:{key}`
/// with the hole loop's OWN stable identity ([`ProfileLoop::key`]), so the name
/// survives another hole being added or removed — the hole-wall sibling of the
/// per-loop cap naming (the old stamp counted holes positionally across regions,
/// so deleting one renumbered the rest). `segment`, when the feature cuts this
/// region once PER PATH SEGMENT (the multi-segment sweep), keys each cut's walls
/// by the segment that made them — otherwise adjacent portions' channel walls
/// would collide on one name and fall back to a positional `[n]`.
pub fn subtract_region_holes(
    mut body: BrepSolid,
    profile: &SketchProfile,
    region: &[crate::feature_pipeline::ProfileLoop],
    feature: &str,
    id: &str,
    segment: Option<&str>,
    build: &mut dyn FnMut(usize, usize) -> Result<BrepSolid, KernelRefusal>,
) -> Result<BrepSolid, FeatureRefusal> {
    for group in hole_nesting(profile, region)? {
        let key = hole_key(&region[group.index], group.index);
        // The cutter's and the cut's refusals keep their class through the
        // feature's wrap.
        let Some(cutter) = hole_cutter(region, &group, id, segment, build).map_err(|error| {
            error.with_message(|error| format!("{feature}: hole {key} cut failed: {error}"))
        })?
        else {
            continue;
        };
        body = subtract_solid(body, cutter).map_err(|error| {
            error.with_message(|error| format!("{feature}: hole {key} cut failed: {error}"))
        })?;
    }
    Ok(body)
}

/// A hole loop's naming key: its own stable loop identity, else its POSITION —
/// the pre-existing spelling, kept for a profile whose loops have no identity to
/// key on (a FACE profile, a hand-built profile), so their names never move.
pub fn hole_key(hole: &crate::feature_pipeline::ProfileLoop, index: usize) -> String {
    hole.key().unwrap_or_else(|| index.to_string())
}

/// Build one hole group's CUTTER: the loop's swept solid minus each child's
/// cutter. `None` for a degenerate (< 2 curve) loop — nothing to cut.
fn hole_cutter(
    region: &[crate::feature_pipeline::ProfileLoop],
    group: &HoleGroup,
    id: &str,
    segment: Option<&str>,
    build: &mut dyn FnMut(usize, usize) -> Result<BrepSolid, KernelRefusal>,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if region[group.index].curves.len() < 2 {
        return Ok(None);
    }
    let mut cutter = build(group.index, group.depth)?;
    name_hole_cutter_faces(
        &mut cutter,
        id,
        &hole_key(&region[group.index], group.index),
        segment,
    );
    for child in &group.children {
        let Some(child_cutter) = hole_cutter(region, child, id, segment, build)? else {
            continue;
        };
        cutter = subtract_solid(cutter, child_cutter).map_err(|error| {
            error.with_message(|error| {
                format!("island {} carve from hole {} failed: {error}", child.index, group.index)
            })
        })?;
    }
    Ok(Some(cutter))
}

// ===========================================================================
// Face frame — a sketch/plane FRAME from a resident BREP face (planar-only)
// ===========================================================================

/// A resolved "plane-like" reference — a solid FACE or a datum/construction PLANE
/// frame. THE shared resolution for a `["FACE","PLANE"]` reference: a datum plane
/// (from a `D`/`P` feature — both emit a scene [`Frame`]) and a planar face are
/// the same underlying object here, so every consumer (mirror, split, pattern,
/// plane) treats them identically instead of hand-rolling the two-table lookup.
/// That duplication was the source of the "picking a datum plane does nothing"
/// bug class — a new consumer just calls [`resolve_plane_reference`].
pub enum PlaneLikeRef {
    /// A solid face — its `(handle, face_id)`.
    Face(FaceRef),
    /// A datum / construction plane — its scene frame.
    Frame(Frame),
}

impl PlaneLikeRef {
    /// The reference's plane as `(point, unit outward normal)`. A datum PLANE is
    /// its frame's `(origin, z-axis)`; a FACE is its midpoint tangent plane
    /// ([`face_point_normal`], valid for any face — a curved face yields the
    /// midpoint tangent, matching the pre-refactor mirror/split behaviour).
    pub fn point_normal(&self) -> Result<(Vec3, Vec3), String> {
        match self {
            PlaneLikeRef::Frame(frame) => Ok((frame.origin, frame.z_axis)),
            PlaneLikeRef::Face(face) => face_point_normal(*face),
        }
    }

    /// The reference's full plane BASIS (origin + in-plane axes + normal). A datum
    /// PLANE is its frame directly; a FACE is [`face_frame`] (planar faces only).
    pub fn frame(&self) -> Result<Frame, String> {
        match self {
            PlaneLikeRef::Frame(frame) => Ok(*frame),
            PlaneLikeRef::Face(face) => face_frame(*face),
        }
    }
}

/// Resolve a `["FACE","PLANE"]` reference name against the live scene-map: a solid
/// FACE first, else a datum / construction PLANE frame. The two name tables are
/// disjoint, so the order is irrelevant. `None` = unresolved (the caller records
/// it as an `unresolved` entry, contract rule 1 — never a no-op-with-no-signal).
pub fn resolve_plane_reference(ctx: &FeatureContext, name: &str) -> Option<PlaneLikeRef> {
    if let Some(face) = ctx.scene.resolve_face(name) {
        return Some(PlaneLikeRef::Face(face));
    }
    ctx.scene.resolve_frame(name).map(PlaneLikeRef::Frame)
}

/// A face's plane as `(point on it, unit outward normal)`: the surface value +
/// normal at its parameter-domain midpoint, oriented by `same_sense`. Works on
/// ANY face (a curved face yields its midpoint tangent plane). The shared version
/// of the `plane_from_face` helper mirror/split/pattern each carried privately.
pub fn face_point_normal(face: FaceRef) -> Result<(Vec3, Vec3), String> {
    crate::with_registered_solid_str(face.handle, |solid| {
        for shell in &solid.shells {
            for record in &shell.faces {
                if record.id != face.face_id {
                    continue;
                }
                let [u0, u1] = record.surface.domain_u()?;
                let [v0, v1] = record.surface.domain_v()?;
                let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
                let point = record.surface.evaluate(um, vm)?;
                let mut normal = record.surface.normal(um, vm)?;
                if !record.same_sense {
                    normal = normal.scale(-1.0);
                }
                return Ok((point, normal));
            }
        }
        Err(format!("face {} not found on solid", face.face_id))
    })
}

/// The world-space BOUNDARY SAMPLES of a face: five points along each
/// non-degenerate edge of each of its loops. This is the face EXTENT as this
/// kernel measures it — [`face_frame`]'s origin is the AABB centre of exactly
/// these points, and the assembly inference lane
/// ([`crate::feature_pipeline::assembly::infer`]) decides whether two faces
/// overlap by projecting them. Empty when every boundary edge is degenerate.
pub fn face_boundary_points(
    solid: &BrepSolid,
    record: &crate::FaceRecord,
) -> Result<Vec<Vec3>, String> {
    let mut points = Vec::new();
    for loop_record in &record.loops {
        for coedge in &loop_record.coedges {
            let Some(edge) = solid.edges.iter().find(|e| e.id == coedge.edge_id) else {
                continue;
            };
            if edge.degenerate {
                continue;
            }
            for step in 0..=4 {
                let t = edge.t0 + (edge.t1 - edge.t0) * (step as f64 / 4.0);
                points.push(edge.curve.evaluate(t)?);
            }
        }
    }
    Ok(points)
}

/// The AABB of a point cloud (`None` when empty).
pub fn bounds_of(points: &[Vec3]) -> Option<(Vec3, Vec3)> {
    let mut iter = points.iter();
    let first = *iter.next()?;
    let (mut min, mut max) = (first, first);
    for point in iter {
        min = Vec3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z));
        max = Vec3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z));
    }
    Some((min, max))
}

/// Compute a placement [`Frame`] from a resident face, for headless sketch/plane
/// resolution against a solid's face. Uses the face-basis convention:
/// the normal is the OUTWARD face normal (sense
/// respected); in-plane axes come from [`Frame::from_origin_normal`]; the origin is
/// the face boundary AABB center projected onto the plane. **Planar faces only** —
/// a curved face is not yet migrated (loud error, no-fallback).
pub fn face_frame(face: FaceRef) -> Result<Frame, String> {
    crate::with_registered_solid_str(face.handle, |solid| {
        let record = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|candidate| candidate.id == face.face_id)
            .ok_or_else(|| format!("face {} not found on solid", face.face_id))?;

        let [u0, u1] = record.surface.domain_u()?;
        let [v0, v1] = record.surface.domain_v()?;
        let (um, vm) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);

        // Planarity: the normal must be constant across the patch.
        let center_normal = record.surface.normal(um, vm)?;
        for (u, v) in [(u0, v0), (u1, v0), (u1, v1), (u0, v1)] {
            let normal = record.surface.normal(u, v)?;
            if normal.dot(center_normal).abs() < 1.0 - 1e-6 {
                return Err(
                    "sketch/plane on a non-planar face is not yet migrated to the Rust pipeline"
                        .into(),
                );
            }
        }
        // Outward face normal (respect the face sense, like `getAverageNormal`).
        let normal = if record.same_sense {
            center_normal
        } else {
            center_normal.scale(-1.0)
        };

        // Boundary AABB center from the face loops' non-degenerate edges.
        let boundary = face_boundary_points(solid, record)?;
        let plane_point = record.surface.evaluate(um, vm)?;
        let center = match bounds_of(&boundary) {
            Some((min, max)) => min.add(max).scale(0.5),
            None => plane_point,
        };
        // Project the AABB center onto the plane so the origin lies exactly on it.
        let unit = normal.normalized()?;
        let signed = center.sub(plane_point).dot(unit);
        let origin = center.sub(unit.scale(signed));
        Frame::from_origin_normal(origin, normal)
    })
}

// ===========================================================================
// Multi-region profiles — union the per-region solids into ONE result
// ===========================================================================

/// UNION the per-region solids of a multi-region profile into ONE solid — the
/// established multi-region behavior (one solid per region, boolean-unioned). Folds with
/// `merge_coplanar_faces` (handle-registered operands, freed after each fold);
/// a single region passes through untouched, so single-region consumers are
/// byte-identical to the pre-region pipeline.
pub fn union_region_solids(solids: Vec<BrepSolid>) -> Result<BrepSolid, String> {
    union_solids_keeping(solids, &[])
}

/// [`union_region_solids`] that PINS the named faces out of the coplanar merge:
/// a face whose name contains one of `keep_unmerged` is never coalesced with a
/// mergeable neighbour.
///
/// The multi-segment sweep needs it. Two portions built from CONSECUTIVE path
/// segments meet at a shared cap, and their sidewalls are frequently coplanar
/// (always so for two collinear segments, and for the walls parallel to the plane
/// a turn happens in). Merging those fuses two portions' walls into one face
/// carrying ONE of the two names — so which of the two source segments a
/// downstream fillet still resolves to would depend on the boolean's internal
/// face order, and would move whenever the path is edited. Pinning keeps one wall
/// face per (path segment × profile edge), each named after both — the same call
/// sheet metal makes to keep a flange attachable to a specific outline segment.
pub fn union_solids_keeping(
    solids: Vec<BrepSolid>,
    keep_unmerged: &[String],
) -> Result<BrepSolid, String> {
    let mut iter = solids.into_iter();
    let mut current = iter
        .next()
        .ok_or("profile produced no region solids to union")?;
    let options = BooleanOptions {
        merge_coplanar_faces: true,
        keep_unmerged_name_substrs: keep_unmerged.to_vec(),
        ..BooleanOptions::default()
    };
    for next in iter {
        let a = crate::register_solid_value(current);
        let b = crate::register_solid_value(next);
        let unioned = crate::with_two_registered_solids(a, b, |left, right| {
            crate::boolean_operation(left, right, BooleanOperation::Union, &options)
        });
        crate::free_registered_solid(a);
        crate::free_registered_solid(b);
        current = unioned.map_err(|error| format!("region union failed: {error}"))?;
    }
    Ok(current)
}


/// Transform controls shared by primitive solid schemas.
pub(super) fn primitive_transform_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "transform",
        "default_value": {
            "position": [
                0,
                0,
                0
            ],
            "rotationEuler": [
                0,
                0,
                0
            ],
            "scale": [
                1,
                1,
                1
            ]
        },
        "referenceSelectionFilter": [
            "FACE",
            "EDGE",
            "VERTEX",
            "PLANE",
            "DATUM"
        ],
        "referenceLabel": "Start Reference",
        "referencePlaceholder": "Select point, edge, or face…",
        "hint": "Select a start reference, then position, rotate, and scale the solid relative to it."
    })
}

/// Optional Boolean controls shared by solid-producing feature schemas.
pub(super) fn optional_boolean_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "boolean_operation",
        "default_value": {
            "targets": [],
            "operation": "NONE",
            "mergeCoplanarFaces": true
        },
        "hint": "Optional boolean operation with selected solids"
    })
}


