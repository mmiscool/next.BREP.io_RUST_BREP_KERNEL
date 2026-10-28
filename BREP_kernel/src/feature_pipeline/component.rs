//! The scene COMPONENT concept.
//!
//! A component is a rigid group of solids inserted by an ACOMP feature (one per
//! placed instance). Its record carries the owning feature id, the library part
//! name, the rigid instance pose, the fixed (grounded) flag, and an opaque source
//! metadata slot; its member solids are ordinary scene-resident solids whose
//! entity names are NAMESPACED with the owning feature id.
//!
//! # Namespacing — the prefix wraps, nothing else moves
//!
//! Every entity name of a member solid (the solid name, every `face.name`, every
//! `edge.name` — the only named entities in this kernel; vertices carry no names)
//! is prefixed with `{component_id}:` AT THE COMPONENT BOUNDARY, i.e. when the
//! sub-part's solids enter the assembly scene:
//!
//! ```text
//!   Extrude1_top            ->  ACOMP2:Extrude1_top
//!   Extrude1|Extrude1_top[0]->  ACOMP2:Extrude1|Extrude1_top[0]
//! ```
//!
//! INSIDE the namespace the deterministic-naming rules apply UNCHANGED: the
//! names arrive final from the sub-part's own `register_added` passes
//! (`ensure_unique_face_names` + `stamp_derived_edge_names`) and are NEVER
//! re-derived here. Re-running the derived-edge pass after prefixing would
//! rewrite `ACOMP2:A|B[0]` from the prefixed face names (different sort, embedded
//! prefixes) — do not "fix" this by routing members through `register_added`.
//! Two instances of the same part therefore never collide (distinct feature ids),
//! and a nested assembly's already-namespaced members chain naturally:
//! `ACOMP1:Extrude1_top` wrapped by `ACOMP3` becomes `ACOMP3:ACOMP1:Extrude1_top`.
//!
//! # The feature fence
//!
//! Solids from ordinary modeling features never belong to a component and behave
//! exactly as before. Component geometry is valid input for constraints and
//! sketch attach/project ONLY; modeling features must cheaply REJECT it —
//! [`SceneMap::is_component_owned`] / [`reject_component_references`] are that
//! predicate (Wave 2 wires the checks into feature resolution).

use std::collections::BTreeMap;

use crate::feature_pipeline::features::common::{collect_edge_names, collect_face_names};
use crate::feature_pipeline::{AddedSolid, FeatureDescriptor, PortRecord, SceneMap};
use crate::{transform_brep, AffineTransform, BrepSolid};

// ===========================================================================
// The component record
// ===========================================================================

/// One scene component: an ACOMP instance's identity, pose, and member solids.
/// Rides the [`crate::feature_pipeline::FeatureResult`] `components` side-channel
/// into `SceneMap::apply`, exactly like profiles/frames — the scene's component
/// set is rebuilt from feature results every history run (a projected view,
/// never an owning data structure).
#[derive(Debug, Clone)]
pub struct ComponentRecord {
    /// The owning ACOMP feature id — also the namespace prefix segment.
    pub id: String,
    /// The display / parts-library part name (`M4-bolt` in `M4-bolt (ACOMP3)`).
    pub part_name: String,
    /// The rigid instance pose (part-local snapshot space -> assembly space),
    /// already baked into the member geometry. Kept so a pose UPDATE can apply
    /// the delta `new · old⁻¹` and so the solver can read the current pose.
    pub transform: AffineTransform,
    /// Grounded flag (the feature's `isFixed`; the solver never moves it).
    pub fixed: bool,
    /// Opaque source metadata slot (`sourceKey` / `sourceSignature` / …) for the
    /// parts-library + update-components lanes; the scene never interprets it.
    pub source: serde_json::Value,
    /// Member solid scene names, already namespaced (`{id}:{part solid name}`).
    pub solids: Vec<String>,
    /// The part's harness PORT ids, namespaced (`{id}:{part port id}`) — the
    /// wire-harness entities a placed part carries. Their records live in
    /// [`SceneMap::ports`]; the re-pose lane moves them with the members.
    pub ports: Vec<String>,
    /// A sub-assembly's own constraint statuses (and those of every level
    /// below it), namespaced with this component's id — solved inside the
    /// sub-assembly's run, reported by the parent's (see `NestedConstraint`).
    pub nested_constraints: Vec<NestedConstraint>,
    /// A sub-assembly's SOLVED occurrence poses: occurrence path below this
    /// component (`ACOMP2`, `ACOMP2:ACOMP1`, …) → its pose in this component's
    /// part-local frame. Its own solve can move an occurrence away from the
    /// transform its document authors, so anything that needs where a nested
    /// occurrence actually sits composes `transform` with this, never the
    /// authored transform. Empty for a part (and for a sub-assembly built from
    /// a snapshot before its first solve this session).
    pub nested_poses: BTreeMap<String, AffineTransform>,
    /// An instance of a Pattern Component (`APAT`): which seed it follows and
    /// how. `None` for every ACOMP-placed component.
    pub pattern: Option<PatternFollow>,
    /// Set on a FLEXIBLE sub-assembly instance (`inputParams.flexible`): what
    /// the parent's solve needs to move its occurrences individually. `None`
    /// = rigid, the default.
    pub flexible: Option<FlexibleSpec>,
}

/// How a pattern instance follows its seed: its pose is `delta · seed pose`
/// (`T · delta · T⁻¹ · seed pose` with `T` the driver's pose, in HOLES mode).
#[derive(Debug, Clone)]
pub struct PatternFollow {
    /// The `APAT` feature that emitted it.
    pub pattern_id: String,
    /// The seed component it follows.
    pub seed: String,
    /// The instance's offset from the seed, in assembly space.
    pub delta: AffineTransform,
    /// Its place in the pattern (the seed is 1), or its hole's key.
    pub key: i64,
    /// HOLES mode: the component whose hole it follows. `delta` is then in
    /// that component's frame and applies through its solved pose.
    pub driver: Option<String>,
}

/// A flexible sub-assembly instance, as the parent's constraint run reads it
/// (`assembly::flexible`): the child document's own constraints (raw, its
/// namespace), which of its occurrences stay with the instance's frame, where
/// the child authors each occurrence, and the per-instance solved poses the
/// placing feature persists. All poses are part-local (the instance frame).
/// `nested` holds the same for each occurrence the child itself places
/// flexible (keyed by its id in the child), at any depth; only the top spec
/// carries seeds and `fixed`, keyed by the full occurrence path
/// (`ACOMP1:ACOMP2`). `fixed` is this instance's own Fix/Float of an
/// occurrence (`inputParams.flexibleFixed`): it wins over the child's
/// grounding, and a fixed occurrence is a grounded body where it stands.
#[derive(Debug, Clone, Default)]
pub struct FlexibleSpec {
    pub constraints: Vec<serde_json::Value>,
    pub grounded: std::collections::BTreeSet<String>,
    pub authored: BTreeMap<String, AffineTransform>,
    pub seeds: BTreeMap<String, AffineTransform>,
    pub fixed: BTreeMap<String, bool>,
    pub nested: BTreeMap<String, FlexibleSpec>,
}

/// One constraint of a NESTED sub-assembly as its parent reports it.
///
/// A sub-assembly is rigid: its own constraints solve inside its own run (the
/// isolated document run that makes its snapshot), and the parent's mates then
/// place it as one body. The parent never re-solves these, but it must still
/// say how they ended, so the row rides the component record. `id` carries the
/// namespace of every level it passed (`ACOMP1:ACOMP2:CONC1`).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NestedConstraint {
    pub id: String,
    #[serde(rename = "type")]
    pub constraint_type: String,
    pub enabled: bool,
    pub status: String,
    pub message: String,
    pub satisfied: bool,
}

impl NestedConstraint {
    /// The row for one constraint the run just settled.
    pub fn from_entry(entry: &crate::feature_pipeline::assembly::ConstraintEntry) -> Self {
        Self {
            id: entry.id().to_string(),
            constraint_type: entry.constraint_type.clone(),
            enabled: entry.enabled,
            status: entry.status().to_string(),
            message: entry
                .persistent("message")
                .and_then(|value| value.as_str())
                .unwrap_or("")
                .to_string(),
            satisfied: entry
                .persistent("satisfied")
                .and_then(|value| value.as_bool())
                .unwrap_or(false),
        }
    }

    /// The same row one level up, under the placing component's id.
    pub fn namespaced(&self, component_id: &str) -> Self {
        Self {
            id: namespaced(component_id, &self.id),
            ..self.clone()
        }
    }

    /// Whether the parent should be told: enabled and not satisfied.
    pub fn is_failure(&self) -> bool {
        self.enabled && !self.satisfied
    }
}

/// The scene's component set, keyed by owning feature id. `BTreeMap` so
/// iteration (structure tree, BOM) is deterministic.
pub type ComponentMap = BTreeMap<String, ComponentRecord>;

// ===========================================================================
// Namespacing
// ===========================================================================

/// Wrap one entity name with a component prefix: `{component_id}:{name}`.
pub fn namespaced(component_id: &str, name: &str) -> String {
    format!("{component_id}:{name}")
}

/// Prefix every NAMED face/edge of a member solid with the component id (the
/// solid's own scene name is prefixed by the caller — solids carry their name in
/// [`AddedSolid`], not on the BREP). Unnamed entities stay unnamed (they are not
/// scene-resolvable either way).
fn namespace_solid_names(solid: &mut BrepSolid, component_id: &str) {
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if let Some(name) = &face.name {
                face.name = Some(namespaced(component_id, name));
            }
        }
    }
    for edge in &mut solid.edges {
        if let Some(name) = &edge.name {
            edge.name = Some(namespaced(component_id, name));
        }
    }
}

/// A component id must be a plain feature id: non-empty, no `:` (the namespace
/// delimiter) and no `|` (the topology-vs-authored name discriminator).
fn validate_component_id(id: &str) -> Result<(), String> {
    if id.is_empty() {
        return Err("component id must not be empty".into());
    }
    if id.contains(':') || id.contains('|') {
        return Err(format!("component id '{id}' must not contain ':' or '|'"));
    }
    Ok(())
}

// ===========================================================================
// Rigid-transform helpers (row-major 4x4, the AffineTransform layout)
// ===========================================================================

/// Require a RIGID map (orthonormal rotation rows, determinant +1). Components
/// move as rigid bodies — a scaling/shearing/reflecting instance pose is a
/// modeling error, rejected loudly (no fallback).
fn require_rigid(transform: &AffineTransform) -> Result<(), String> {
    let m = &transform.elements;
    let rows = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]];
    for i in 0..3 {
        for j in i..3 {
            let dot: f64 = (0..3).map(|k| rows[i][k] * rows[j][k]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            if (dot - expected).abs() > 1e-8 {
                return Err("component transform must be rigid (rotation + translation)".into());
            }
        }
    }
    if (transform.determinant3() - 1.0).abs() > 1e-8 {
        return Err("component transform must be rigid (no reflection/scale)".into());
    }
    Ok(())
}

/// Row-major product `a · b` (apply `b` first, then `a`).
pub(crate) fn compose(a: &AffineTransform, b: &AffineTransform) -> Result<AffineTransform, String> {
    let (ma, mb) = (&a.elements, &b.elements);
    let mut out = [0.0f64; 16];
    for row in 0..4 {
        for col in 0..4 {
            out[row * 4 + col] = (0..4)
                .map(|k| ma[row * 4 + k] * mb[k * 4 + col])
                .sum();
        }
    }
    AffineTransform::new(out)
}

// ===========================================================================
// Create / re-pose
// ===========================================================================

/// Build a component from a set of PART-LOCAL solids: rigid-pose each member
/// with `transform`, namespace its entity names with `id`, and register it as a
/// scene-resident solid. Returns the component record plus one [`AddedSolid`]
/// per member — the owning ACOMP feature pushes both into its `FeatureResult`
/// (`added` + `components`), and the record's handles are then owned by that
/// feature's history-cache entry like any other feature output.
///
/// `members` are `(solid name, part-local BREP)` pairs exactly as the sub-part's
/// history produced them (deterministic names final; see the module doc — the
/// prefix wraps them, nothing is re-derived). Two-phase: every member is posed
/// BEFORE anything registers, so a failure never leaks a half-built component.
// Contract surface for the Wave-2 ACOMP feature; exercised by this module's tests.
#[allow(dead_code)]
pub fn create_component(
    id: &str,
    part_name: &str,
    fixed: bool,
    source: serde_json::Value,
    transform: AffineTransform,
    members: Vec<(String, BrepSolid)>,
) -> Result<(ComponentRecord, Vec<AddedSolid>), String> {
    validate_component_id(id)?;
    require_rigid(&transform)?;

    // Phase 1 (fallible): pose + namespace every member.
    let mut posed: Vec<(String, BrepSolid)> = Vec::with_capacity(members.len());
    for (member_name, solid) in &members {
        let mut body = transform_brep(solid, transform, false)
            .map_err(|error| format!("component '{id}': member '{member_name}': {error}"))?;
        namespace_solid_names(&mut body, id);
        posed.push((namespaced(id, member_name), body));
    }

    // Phase 2 (infallible): register.
    let mut added = Vec::with_capacity(posed.len());
    let mut solids = Vec::with_capacity(posed.len());
    for (name, body) in posed {
        let face_names = collect_face_names(&body);
        let edge_names = collect_edge_names(&body);
        let handle = crate::register_solid_value(body);
        solids.push(name.clone());
        added.push(AddedSolid {
            handle,
            name,
            face_names,
            edge_names,
            ..AddedSolid::default()
        });
    }

    Ok((
        ComponentRecord {
            id: id.to_string(),
            part_name: part_name.to_string(),
            transform,
            fixed,
            source,
            solids,
            ports: Vec::new(),
            nested_constraints: Vec::new(),
            nested_poses: BTreeMap::new(),
            pattern: None,
            flexible: None,
        },
        added,
    ))
}

/// Pose a part-local port record by a rigid transform: the base point maps as
/// a point, the direction by the rotation part alone.
pub fn pose_port(record: &PortRecord, transform: &AffineTransform) -> PortRecord {
    let point = transform.point(record.point);
    let tip = transform.point(record.point.add(record.direction));
    let direction = tip.sub(point).normalized().unwrap_or(record.direction);
    PortRecord {
        point,
        direction,
        ..record.clone()
    }
}

/// Attach a part's connection points to a placed component: each point's
/// ADDRESS is wrapped `{component}:{address}` and its record is posed by the
/// component transform. The record's `port_name` / `point_name` / `purpose`
/// stay PART-LOCAL — the occurrence chain lives in the address, so pin `VCC` of
/// port `J1` reads the same however deep it is placed. The record lists the
/// addresses so [`update_component_transform`] moves them with the members; the
/// posed records are returned for the feature to publish.
pub fn attach_component_ports(
    record: &mut ComponentRecord,
    ports: &BTreeMap<String, PortRecord>,
) -> Vec<(String, PortRecord)> {
    let mut posed = Vec::with_capacity(ports.len());
    for (local_id, port) in ports {
        let id = namespaced(&record.id, local_id);
        let port = pose_port(port, &record.transform);
        record.ports.push(id.clone());
        posed.push((id, port));
    }
    posed
}

/// Re-pose a component to a NEW absolute rigid transform: applies the delta
/// `new · old⁻¹` to every member solid IN PLACE (same handles — a rigid map
/// keeps every topology id and name, so the scene's face/edge refs stay valid)
/// and updates the record.
///
/// This is the INTERACTIVE lane (move-gizmo drag, solver preview). The
/// AUTHORITATIVE pose lives on the owning ACOMP feature's `inputParams`
/// transform: a commit must write it back there (the solver pose write-back),
/// which dirties the feature's fingerprint so the next history run re-executes
/// it from the snapshot at the committed pose. Without the write-back, a clean
/// cache replay would serve the mutated geometry against a stale descriptor.
///
/// Two-phase: every member transform is computed before anything commits, so a
/// failure never leaves the component half-posed.
// Contract surface for the Wave-2/3 move + solver lanes; exercised by tests.
#[allow(dead_code)]
pub fn update_component_transform(
    scene: &mut SceneMap,
    id: &str,
    new_transform: AffineTransform,
) -> Result<(), String> {
    require_rigid(&new_transform)?;
    let record = scene
        .resolve_component(id)
        .ok_or_else(|| format!("unknown component '{id}'"))?;
    let delta = compose(&new_transform, &record.transform.rigid_inverse()?)?;
    let port_ids = record.ports.clone();

    // Phase 1 (fallible): resolve + re-pose every member.
    let mut posed: Vec<(u32, BrepSolid)> = Vec::with_capacity(record.solids.len());
    for name in &record.solids {
        let handle = scene
            .solids
            .get(name)
            .copied()
            .ok_or_else(|| format!("component '{id}': member '{name}' is not scene-resident"))?;
        let body = crate::with_registered_solid_str(handle, |solid| {
            transform_brep(solid, delta, false)
        })
        .map_err(|error| format!("component '{id}': member '{name}': {error}"))?;
        posed.push((handle, body));
    }

    // Phase 2: commit under the existing handles, then update the record.
    for (handle, body) in posed {
        crate::replace_registered_solid(handle, body)?;
    }
    if let Some(record) = scene.components.get_mut(id) {
        record.transform = new_transform;
        // A flexible instance's occurrences are among its members: they moved
        // with it, so their frames follow.
        let prefix = format!("{id}:");
        for occurrence in scene.occurrences.values_mut().filter(|o| o.id.starts_with(&prefix)) {
            occurrence.transform = compose(&delta, &occurrence.transform)?;
        }
        super::mark_component_cache_posed(id);
    } else if scene.occurrences.contains_key(id) {
        // A flexible occurrence placed inside a flexible occurrence moved
        // with it too.
        let prefix = format!("{id}:");
        for occurrence in scene.occurrences.values_mut() {
            if occurrence.id == id {
                occurrence.transform = new_transform;
            } else if occurrence.id.starts_with(&prefix) {
                occurrence.transform = compose(&delta, &occurrence.transform)?;
            }
        }
        // The placing feature's cache entry owns the occurrence's solids.
        let owner = id.split(':').next().unwrap_or(id).to_string();
        super::mark_component_cache_posed(&owner);
    }

    // The component's ports (and their published entities) follow the members,
    // so a solve that moves a part moves the harness network it carries.
    for port_id in port_ids {
        let Some(port) = scene.ports.get(&port_id) else {
            continue;
        };
        let posed = pose_port(port, &delta);
        crate::feature_pipeline::ports::place_scene_port(scene, &port_id, posed)?;
    }
    Ok(())
}

/// The feature fence: fail with a clear message when any of `names` resolves to
/// component-owned geometry. Modeling features that consume solids/faces/edges
/// call this over their reference names before operating — cross-part feature
/// linking is out of scope and must be rejected cleanly, never silently
/// applied.
pub fn reject_component_references<'a>(
    scene: &SceneMap,
    names: impl IntoIterator<Item = &'a str>,
) -> Result<(), String> {
    for name in names {
        if let Some(record) = scene.owning_component(name) {
            return Err(format!(
                "'{name}' belongs to assembly component '{}' — modeling features cannot consume component geometry",
                record.id
            ));
        }
    }
    Ok(())
}

/// The fence at the DISPATCH altitude — the one place every feature passes
/// through (`execute_feature`), so no per-feature checks are scattered. A
/// reference here is what the pipeline already defines it to be (the
/// `scan_consumed` cache-dependency walk): any descriptor string that
/// exact-matches a scene name. Any such string owned by a component fails the
/// feature before it executes.
///
/// Exempt (the two ALLOWED in-context uses, attach + project): SKETCH (plane
/// attach + edge projection), DATUM/PLANE (a datum derived from component
/// geometry), ACOMP itself (the component's own feature, wired by the
/// parts-library lane), and the two wire-harness construction features —
/// WAYPOINT (its `directionRef` is a face on a placed component: the harness
/// workbench's whole point) and SPLINE (its anchors attach to connection
/// points). Assembly constraints are not history features, so they never
/// reach this fence. The exemption is type-level, which is safe because none
/// of the exempt types carries a `boolean` param or operand-solid references
/// — re-examine if such a param is ever added to one of them.
pub fn enforce_reference_fence(
    feature_type: &str,
    descriptor: &FeatureDescriptor,
    scene: &SceneMap,
) -> Result<(), String> {
    // Componentless scenes skip the walk entirely — the byte-identical
    // guarantee for every existing modeling document.
    if scene.components.is_empty() {
        return Ok(());
    }
    // Verbatim dispatch alias strings (the dispatch matches exact spellings;
    // "ASSEMBLY COMPONENT" is ACOMP's long dispatch alias in assembly_component.rs).
    if matches!(
        feature_type,
        "S" | "SKETCH"
            | "S3D"
            | "SKETCH3D"
            | "3D SKETCH"
            | "D"
            | "DATUM"
            | "DATIUM"
            | "P"
            | "PLANE"
            | "ACOMP"
            | "ASSEMBLY COMPONENT"
            | "APAT"
            | "PATTERN COMPONENT"
            | "WP"
            | "WAYPOINT"
            | "SP"
            | "SPLINE"
    ) {
        return Ok(());
    }
    if let Some(name) = first_component_owned(&descriptor.input_params, scene)
        .or_else(|| first_component_owned(&descriptor.persistent_data, scene))
    {
        return reject_component_references(scene, [name]);
    }
    Ok(())
}

/// The first string anywhere in `value` (recursively, both param sources — the
/// `scan_consumed` walk shape) that names component-owned geometry.
fn first_component_owned<'a>(
    value: &'a serde_json::Value,
    scene: &SceneMap,
) -> Option<&'a str> {
    match value {
        serde_json::Value::String(text) => {
            let trimmed = text.trim();
            (!trimmed.is_empty() && scene.is_component_owned(trimmed)).then_some(trimmed)
        }
        serde_json::Value::Array(items) => {
            items.iter().find_map(|item| first_component_owned(item, scene))
        }
        serde_json::Value::Object(map) => {
            map.values().find_map(|item| first_component_owned(item, scene))
        }
        _ => None,
    }
}

// ===========================================================================
// SceneMap component surface
// ===========================================================================

impl SceneMap {
    /// Resolve a component record by its owning feature id (exact match).
    /// Contract surface for the Wave-2 constraint/selection lanes.
    #[allow(dead_code)]
    pub fn resolve_component(&self, id: &str) -> Option<&ComponentRecord> {
        self.components.get(id).or_else(|| self.occurrences.get(id))
    }

    /// The component owning an entity name, if any: the BARE component id itself
    /// (a COMPONENT-type selection), or the name's OUTERMOST namespace prefix
    /// (`ACOMP3:ACOMP1:X` is owned by `ACOMP3` at this scene level; `ACOMP1` is
    /// the nested assembly's internal structure). Membership is decided against
    /// the registered component set, never syntactically — authored names like
    /// `S1:PROFILE` have a first segment that is a sketch id, not a component id.
    pub fn owning_component(&self, name: &str) -> Option<&ComponentRecord> {
        if let Some(record) = self.resolve_component(name) {
            return Some(record);
        }
        // A flexible instance's occurrence owns its own geometry (the deepest
        // registered prefix wins over the instance).
        if let Some(record) = self
            .occurrences
            .values()
            .filter(|record| name.starts_with(&format!("{}:", record.id)))
            .max_by_key(|record| record.id.len())
        {
            return Some(record);
        }
        let (prefix, _) = name.split_once(':')?;
        self.components.get(prefix)
    }

    /// The feature-fence predicate: does this entity name belong to a component?
    /// Cheap (one map probe + one prefix probe); the dispatch fence
    /// (`enforce_reference_fence`) uses it to REJECT component geometry as
    /// modeling-feature input.
    pub fn is_component_owned(&self, name: &str) -> bool {
        self.owning_component(name).is_some()
    }

    /// Iterate the scene's components in deterministic (id) order — the
    /// structure-tree / BOM projection surface.
    #[allow(dead_code)]
    pub fn iter_components(&self) -> impl Iterator<Item = &ComponentRecord> {
        self.components.values()
    }

    /// A component's member solids as `(scene name, resident handle)`, in the
    /// record's member order. A member missing from the solid map (never the
    /// case for an intact scene) is skipped.
    #[allow(dead_code)]
    pub fn component_solids(&self, id: &str) -> Vec<(String, u32)> {
        let Some(record) = self.resolve_component(id) else {
            return Vec::new();
        };
        record
            .solids
            .iter()
            .filter_map(|name| self.solids.get(name).map(|&handle| (name.clone(), handle)))
            .collect()
    }

    /// The member solids of an occurrence path: a component's (or a flexible
    /// occurrence's) own record, else every solid named under the path, which
    /// is how a placement inside a rigid sub-assembly is reached
    /// (`ACOMP1:ACOMP2` → `ACOMP1:ACOMP2:Pin`), in name order.
    pub fn occurrence_solids(&self, path: &str) -> Vec<(String, u32)> {
        if self.resolve_component(path).is_some() {
            return self.component_solids(path);
        }
        let prefix = format!("{path}:");
        let mut solids: Vec<(String, u32)> = self
            .solids
            .iter()
            .filter(|(name, _)| name.starts_with(&prefix))
            .map(|(name, &handle)| (name.clone(), handle))
            .collect();
        solids.sort();
        solids
    }

    /// A component's grounded flag (`None` for an unknown id).
    #[allow(dead_code)]
    pub fn component_fixed(&self, id: &str) -> Option<bool> {
        self.resolve_component(id).map(|record| record.fixed)
    }

    /// Set a component's grounded flag. Scene-view state only — the
    /// authoritative flag is the owning feature's `isFixed` (the Fixed
    /// constraint / tree action writes THERE; this keeps the live scene in
    /// sync between runs). Returns false for an unknown id.
    #[allow(dead_code)]
    pub fn set_component_fixed(&mut self, id: &str, fixed: bool) -> bool {
        match self.components.get_mut(id).or(self.occurrences.get_mut(id)) {
            Some(record) => {
                record.fixed = fixed;
                true
            }
            None => false,
        }
    }
}

