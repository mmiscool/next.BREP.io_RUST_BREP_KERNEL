//! The CAM pane — the browser tree of the CAM workbench
//! ([`crate::workbench::cam`]): **Setups**, each with its operations, **NC
//! Programs** and the **Tool Library**, over the document's `cam` block.
//!
//! The pane holds only the widget's transient state (its hit rects, the
//! selected row, an action a toolbar button handed it). Every record it draws
//! is read from the engine's document each frame through
//! [`crate::document::cam`], which is the one place the block's shape is
//! spelled, and every edit is one checkpointed write of the whole block
//! ([`History::set_cam_block`](brep_render::history::History::set_cam_block)) — undo and save / reload come from the
//! document, not from here.
//!
//! A block the document module cannot read is left exactly as it is: the
//! pane shows why under its header and refuses every mutation until the
//! document is repaired, so nothing a user wrote is ever saved over.

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::cam::{self as cam_doc, resolve_frame, strategy_kinds, CamDocument, SetupFrame, SetupOrientation};
use crate::form_view::{form_view, FormViewSpec};
use crate::panels::file::{SaveBody, SaveRequest};
use crate::panels::{cam_generate, cam_library, cam_machine, cam_operation, cam_setup, cam_tool};
use crate::store::ModelStore;
use brep_cam::model::{Point3, RigidTransform};
use brep_render::engine_state::{EngineState, RefSelectTarget};
use eframe::egui;
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashMap};

/// An action a toolbar button hands the pane (the shell's dispatch arms in
/// `app.rs`), performed on the next frame the pane draws — which the shell
/// guarantees by surfacing the pane in the same arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamRequest {
    NewSetup,
    NewTool,
    NewOperation,
    Generate,
}

/// The document's `cam` block, or why it cannot be read.
pub fn cam_document(state: &EngineState) -> Result<CamDocument, String> {
    cam_doc::load(state.history.cam_block())
}

/// Write the block back as a USER edit (one undo step). `coalesce` groups a
/// run of rapid same-target edits (typing in one field) into one step.
pub fn write_cam_document(state: &mut EngineState, doc: &CamDocument, coalesce: Option<&str>) {
    state.history.set_cam_block(cam_doc::store(doc), coalesce);
    state.dirty = true;
}

/// Write the block back WITHOUT an undo step — for a derived value that
/// belongs to the edit just made (the frame a pick resolves to).
fn write_cam_document_no_undo(state: &mut EngineState, doc: &CamDocument) {
    state.history.set_cam_block_no_undo(cam_doc::store(doc));
    state.dirty = true;
}

/// The frame setup `id`'s picks resolve to against the model on screen, or
/// why they do not.
pub fn setup_frame(state: &EngineState, doc: &CamDocument, id: &str) -> Result<SetupFrame, String> {
    let orientation = doc.orientation(id);
    let face = |name: &str| cam_doc::classify_face(&face_triangles(state, name));
    // A feature's published axis (a helix's `…:Axis`) reads as a straight
    // edge along it.
    let edge = |name: &str| {
        state.scene.edge_polyline_world(name).or_else(|| {
            state.cam_named_axis(name).map(|(p, d)| vec![p, [p[0] + d[0], p[1] + d[1], p[2] + d[2]]])
        })
    };
    resolve_frame(&orientation, &face, &edge)
}

/// The world triangles of the face named `name` (empty when no solid on
/// screen carries it), for the orientation's face classifier.
fn face_triangles(state: &EngineState, name: &str) -> Vec<[[f64; 3]; 3]> {
    for solid in state.scene.solids() {
        let Some(face) = solid.faces.iter().find(|f| f.name == name) else { continue };
        let (positions, indices) = (&solid.mesh.positions, &solid.mesh.indices);
        let start = face.tri_start as usize;
        let end = (start + face.tri_count as usize).min(indices.len() / 3);
        let p = |i: u32| {
            let v = positions[i as usize];
            [v[0] as f64, v[1] as f64, v[2] as f64]
        };
        return (start..end).map(|t| [p(indices[t * 3]), p(indices[t * 3 + 1]), p(indices[t * 3 + 2])]).collect();
    }
    Vec::new()
}

/// Apply a form PATCH (`cam_setup::apply_setup_form`) to setup `id`:
/// one undo step, coalesced per setup and field run when `coalesce` is set.
pub fn update_setup(state: &mut EngineState, id: &str, patch: &Value, coalesce: Option<&str>) -> Result<(), String> {
    edit_cam_document(state, coalesce, |doc| {
        let mut orientation = doc.orientation(id);
        let setup = doc.setup_mut(id).ok_or_else(|| format!("no setup `{id}`"))?;
        cam_setup::apply_setup_form(patch, setup, &mut orientation)?;
        // A rest stock starts from an EARLIER setup's simulated stock.
        if let brep_cam::model::Stock::FromRest { setup: source } = &setup.stock {
            let source = source.as_str().to_string();
            let at = |name: &str| doc.setups.iter().position(|s| s.id.as_str() == name);
            match (at(&source), at(id)) {
                (Some(s), Some(me)) if s < me => {}
                (Some(_), _) => return Err(format!("stockSetup: setup `{source}` comes after `{id}`; a rest stock starts from an earlier setup")),
                (None, _) => return Err(format!("stockSetup: no setup `{source}`")),
            }
        }
        doc.set_orientation(id, orientation);
        cam_setup::apply_machine_form(patch, doc, id)?;
        Ok(())
    })?;
    sync_setup_frame(state, id);
    Ok(())
}

/// Write the frame setup `id`'s picks resolve to into its `modelToMachine`
/// when it differs from what is stored — the derived half of a pick or an
/// orientation edit, written without its own undo step. Picks that do not
/// resolve leave the stored transform alone; the dialog shows why.
pub fn sync_setup_frame(state: &mut EngineState, id: &str) -> Option<String> {
    let doc = cam_document(state).ok()?;
    let frame = match setup_frame(state, &doc, id) {
        Ok(frame) => frame,
        Err(why) => return Some(why),
    };
    let transform = match frame.transform() {
        Ok(t) => t,
        Err(why) => return Some(why),
    };
    let setup = doc.setup(id)?;
    if setup.model_to_machine == transform {
        return None;
    }
    let mut next = doc.clone();
    next.setup_mut(id)?.model_to_machine = transform;
    write_cam_document_no_undo(state, &next);
    None
}

/// Apply a form PATCH (`cam_tool::apply_tool_form`) to tool `id`: one undo
/// step, coalesced per tool and field run when `coalesce` is set.
pub fn update_tool(state: &mut EngineState, id: &str, patch: &Value, coalesce: Option<&str>) -> Result<(), String> {
    edit_cam_document(state, coalesce, |doc| {
        let tool = doc.tool_mut(id).ok_or_else(|| format!("no tool `{id}`"))?;
        cam_tool::apply_tool_form(patch, tool)
    })
}

/// Add a custom machine copied from `base` (a built-in id, or another custom
/// machine's), named `name`. Returns the id.
pub fn add_machine(state: &mut EngineState, base: &str, name: Option<&str>) -> Result<String, String> {
    edit_cam_document(state, None, |doc| {
        let source = doc.machine(base).ok_or_else(|| format!("base: no machine `{base}` (a built-in id or a document machine)"))?;
        Ok(doc.add_machine(source, name))
    })
}

/// Apply a machine editor PATCH (`cam_machine::apply_machine_form`) to the
/// document's machine `id`; a built-in is copied first (`add_machine`).
pub fn update_machine(state: &mut EngineState, id: &str, patch: &Value, coalesce: Option<&str>) -> Result<(), String> {
    edit_cam_document(state, coalesce, |doc| {
        let machine = doc.machine_mut(id).ok_or_else(|| format!("no document machine `{id}` (a built-in is copied into the document first)"))?;
        cam_machine::apply_machine_form(patch, machine)
    })
}

/// Duplicate tool `id` under a fresh id and the next free number. Returns
/// the new id.
pub fn duplicate_tool(state: &mut EngineState, id: &str) -> Result<String, String> {
    edit_cam_document(state, None, |doc| {
        let source = doc.tool(id).ok_or_else(|| format!("no tool `{id}`"))?.clone();
        let number = doc.tools.iter().map(|t| t.number).max().unwrap_or(0) + 1;
        let new_id = doc.next_id("T");
        doc.tools.push(cam_tool::duplicate(&source, &new_id, number));
        Ok(new_id)
    })
}

/// Feeds and speeds for tool `id` from `material`, through C12's
/// calculator, written into the tool. Returns the calculator's adjustments.
pub fn calculate_tool_feeds(state: &mut EngineState, id: &str, material: &str) -> Result<Vec<String>, String> {
    edit_cam_document(state, None, |doc| {
        let tool = doc.tool_mut(id).ok_or_else(|| format!("no tool `{id}`"))?;
        cam_tool::calculate_feeds(tool, material)
    })
}

/// Add user-library tool `library_id` to the document under a fresh id and
/// the next free number (its name and geometry as the library has them).
pub fn add_tool_from_library(state: &mut EngineState, store: &dyn ModelStore, library_id: &str) -> Result<String, String> {
    let source = cam_library::tool(store, library_id)?;
    edit_cam_document(state, None, |doc| {
        let number = doc.tools.iter().map(|t| t.number).max().unwrap_or(0) + 1;
        let id = doc.next_id("T");
        let mut tool = source.clone();
        tool.id = id.as_str().into();
        tool.number = number;
        if let Some(holder) = &mut tool.holder {
            holder.id = format!("{id}-holder").into();
        }
        doc.tools.push(tool);
        Ok(id)
    })
}

/// Import a library file's text (`cam_library::import_text`) into the
/// document's tools, each under a fresh id and number. Returns the ids added
/// and the importer's skipped rows.
pub fn import_tools(state: &mut EngineState, name: &str, text: &str) -> Result<(Vec<String>, Vec<String>), String> {
    let report = cam_library::import_text(name, text)?;
    let issues: Vec<String> = report.issues.iter().map(|i| format!("row {}: {}", i.row, i.message)).collect();
    let ids = edit_cam_document(state, None, |doc| {
        let mut ids = Vec::new();
        let mut number = doc.tools.iter().map(|t| t.number).max().unwrap_or(0);
        for mut tool in report.document.library.tools.clone() {
            number += 1;
            let id = doc.next_id("T");
            tool.id = id.as_str().into();
            tool.number = number;
            if let Some(holder) = &mut tool.holder {
                holder.id = format!("{id}-holder").into();
            }
            doc.tools.push(tool);
            ids.push(id);
        }
        Ok(ids)
    })?;
    Ok((ids, issues))
}

/// The document's tools as a library document, for export.
pub fn export_document_tools(state: &EngineState) -> Result<String, String> {
    let doc = cam_document(state)?;
    let mut library = brep_cam::library::ToolLibrary::new("Document tools");
    library.tools = doc.tools.clone();
    brep_cam::library::interchange::LibraryDocument::new(library).to_json()
}

/// Apply a form PATCH (`cam_operation::apply_operation_form`) to operation
/// `id`: one undo step, coalesced per operation and field run when
/// `coalesce` is set. Face references the patch names get their owning
/// solid from the scene.
pub fn update_operation(state: &mut EngineState, id: &str, patch: &Value, coalesce: Option<&str>) -> Result<(), String> {
    let owners = face_owners(state);
    let edge_owners = edge_owners(state);
    edit_cam_document(state, coalesce, |doc| {
        let tools = doc.tools.clone();
        // The multi-axis finish's source designation (lane C17), refused by
        // field before anything is applied.
        crate::panels::cam_multiaxis::apply_source(patch, doc, id)?;
        let operation = doc.operation_mut(id).ok_or_else(|| format!("no operation `{id}`"))?;
        cam_operation::apply_operation_form(patch, operation, &tools)?;
        cam_operation::resolve_face_owners(operation, &|face| owners.get(face).cloned());
        cam_operation::resolve_edge_owners(operation, &|edge| edge_owners.get(edge).cloned());
        Ok(())
    })
}

/// Every edge name on screen → its solid's name.
fn edge_owners(state: &EngineState) -> HashMap<String, String> {
    let mut owners = HashMap::new();
    for solid in state.scene.solids() {
        for edge in solid.edges.iter().filter(|e| !e.name.is_empty()) {
            owners.insert(edge.name.clone(), solid.name.clone());
        }
    }
    owners
}

/// Every face name on screen → its solid's name.
fn face_owners(state: &EngineState) -> HashMap<String, String> {
    let mut owners = HashMap::new();
    for solid in state.scene.solids() {
        for face in &solid.faces {
            owners.insert(face.name.clone(), solid.name.clone());
        }
    }
    owners
}

/// Fill the owning solid of operation `id`'s picked faces from the scene —
/// the derived half of a face pick, written without its own undo step — and
/// sample a committed machining-boundary pick into its containment boundary.
/// A boundary pick that cannot be used (not a sketch profile, a plane not
/// across the setup's Z) is consumed and refused by name; the boundary stays.
pub fn sync_operation_faces(state: &mut EngineState, id: &str) -> Result<(), String> {
    let owners = face_owners(state);
    let edge_owners = edge_owners(state);
    // A committed chain pick is folded into the model by the load; the block
    // is rewritten once so it no longer carries the staging key.
    let block = state.history.cam_block();
    let pending = block.as_ref().is_some_and(|b| cam_doc::has_pending_chain_pick(b));
    let boundary_pick = block.as_ref().and_then(|b| cam_doc::pending_boundary_pick(b, id));
    let Ok(mut doc) = cam_document(state) else { return Ok(()) };
    let setup = doc.operation(id).and_then(|op| doc.setup(op.setup.as_str())).cloned();
    let tools = doc.tools.clone();
    let Some(operation) = doc.operation_mut(id) else { return Ok(()) };
    let faces = cam_operation::resolve_face_owners(operation, &|face| owners.get(face).cloned());
    let edges = cam_operation::resolve_edge_owners(operation, &|edge| edge_owners.get(edge).cloned());
    let mut refused = None;
    if let Some(names) = &boundary_pick {
        let chord = operation.tolerance / 4.0;
        let sampled = setup
            .as_ref()
            .ok_or_else(|| format!("{id}: its setup is not defined"))
            .and_then(|setup| boundary_from_profile(state, names, setup, chord))
            .and_then(|loops| {
                let loops: Vec<Value> = loops.iter().map(|l| json!(l.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>())).collect();
                cam_operation::apply_operation_form(&json!({ "strategy": { "containment": { "boundary": loops } } }), operation, &tools).map_err(|e| format!("{id}: {e}"))
            });
        refused = sampled.err();
    }
    if faces || edges || pending || boundary_pick.is_some() {
        write_cam_document_no_undo(state, &doc);
    }
    refused.map_or(Ok(()), Err)
}

/// The loops of the sketch profile `names` picks (every region, outer and
/// holes — the containment reads loops even-odd), sampled within `chord` and
/// taken into the machine XY of `setup`. Refused by name for anything but one
/// sketch profile, and for a profile whose plane is not across machine Z.
fn boundary_from_profile(state: &EngineState, names: &[String], setup: &brep_cam::model::Setup, chord: f64) -> Result<Vec<Vec<brep_cam::model::Point2>>, String> {
    use brep_cam::model::{ChainSource, Point2};
    let chain = cam_doc::chain_from_names(names).ok_or_else(|| "nothing was picked for the machining boundary".to_string())?;
    if !matches!(chain.source, ChainSource::SketchProfile { .. }) {
        return Err(format!("the machining boundary is picked as a sketch profile (a sketch's face), not `{}`", names.join(", ")));
    }
    let (_, profiles) = state.cam_chain_geometry(std::slice::from_ref(&chain))?;
    let (sketch, profile) = profiles.first().ok_or_else(|| "the picked sketch has no profile".to_string())?;
    let mut loops = Vec::new();
    for region in &profile.regions {
        for profile_loop in region {
            let mut points: Vec<Point3> = Vec::new();
            for curve in &profile_loop.curves {
                for p in sample_curve(curve, chord)? {
                    let p = setup.model_to_machine.apply_point(Point3::new(p.x, p.y, p.z));
                    if points.last().is_none_or(|q: &Point3| q.distance(p) > 1e-9) {
                        points.push(p);
                    }
                }
            }
            if points.len() > 1 && points[0].distance(*points.last().expect("non-empty")) <= 1e-9 {
                points.pop();
            }
            let (lo, hi) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p.z), hi.max(p.z)));
            if hi - lo > chord.max(1e-6) {
                return Err(format!("sketch `{sketch}` is not across the setup's Z (its profile spans {:.4} in machine Z); pick a profile parallel to the machine's XY", hi - lo));
            }
            loops.push(points.iter().map(|p| Point2::new(p.x, p.y)).collect());
        }
    }
    if loops.is_empty() {
        return Err(format!("sketch `{sketch}` has no closed profile"));
    }
    Ok(loops)
}

/// Points along `curve` whose chords stay within `chord` of it: a line its
/// two ends; a curved span split until each chord's midpoint deviation is
/// within `chord` (16 halvings at most).
fn sample_curve(curve: &brep_render::brep_kernel::NurbsCurve, chord: f64) -> Result<Vec<brep_render::brep_kernel::Vec3>, String> {
    use brep_render::brep_kernel::Vec3;
    let [a, b] = curve.domain()?;
    let at = |t: f64| curve.evaluate(t);
    let dist = |p: Vec3, q: Vec3, m: Vec3| {
        let (dx, dy, dz) = (q.x - p.x, q.y - p.y, q.z - p.z);
        let (mx, my, mz) = (m.x - p.x, m.y - p.y, m.z - p.z);
        let len2 = dx * dx + dy * dy + dz * dz;
        let t = if len2 > 0.0 { ((mx * dx + my * dy + mz * dz) / len2).clamp(0.0, 1.0) } else { 0.0 };
        ((mx - t * dx).powi(2) + (my - t * dy).powi(2) + (mz - t * dz).powi(2)).sqrt()
    };
    let mut out = vec![at(a)?];
    // A curved span starts at eight pieces, so an S-bend's chord midpoint
    // cannot sit on the curve by chance.
    let pieces = if curve.degree <= 1 { (curve.control_points.len() - 1).max(1) } else { 8 * (curve.control_points.len() - curve.degree).max(1) };
    let mut stack: Vec<(f64, f64, u32)> = (0..pieces).rev().map(|i| (a + (b - a) * i as f64 / pieces as f64, a + (b - a) * (i + 1) as f64 / pieces as f64, 0)).collect();
    while let Some((t0, t1, depth)) = stack.pop() {
        let (p0, p1, pm) = (at(t0)?, at(t1)?, at(0.5 * (t0 + t1))?);
        if curve.degree > 1 && depth < 16 && dist(p0, p1, pm) > chord {
            stack.push((0.5 * (t0 + t1), t1, depth + 1));
            stack.push((t0, 0.5 * (t0 + t1), depth + 1));
        } else {
            out.push(p1);
        }
    }
    Ok(out)
}

/// Arm the reference picker for operation `id`'s field `key`; refused for a
/// key that is not one of its reference fields.
pub fn begin_operation_pick(state: &mut EngineState, id: &str, key: &str) -> Result<(), String> {
    let doc = cam_document(state)?;
    let operation = doc.operation(id).ok_or_else(|| format!("no operation `{id}`"))?;
    let (path, filter, multiple, label) = cam_operation::pick_path(id, key).ok_or_else(|| format!("`{key}` is not a reference field of an operation"))?;
    let params = cam_operation::operation_form_params(operation);
    let seed: Vec<String> = params["targets"].get(key).and_then(Value::as_array).map(|l| l.iter().filter_map(Value::as_str).map(str::to_string).collect()).unwrap_or_default();
    if state.ref_select_active() {
        state.finish_ref_select();
    }
    state.begin_ref_select_for_cam(id, path, format!("{}: {label}", operation.name), filter, multiple, seed);
    Ok(())
}

/// Queue the generation of `ops` (every enabled operation with targets when
/// `None`): the jobs the engine's runner takes, one operation each. Returns
/// the ids queued and, per operation that could not be queued, why.
pub fn generate(state: &mut EngineState, ops: Option<&[String]>) -> Result<(Vec<String>, Vec<(String, String)>), String> {
    let doc = cam_document(state)?;
    let ids: Vec<String> = match ops {
        Some(ids) => ids.to_vec(),
        None => doc.operations.iter().filter(|op| op.enabled).map(|op| op.id.as_str().to_string()).collect(),
    };
    let mut jobs = Vec::new();
    let mut refused = Vec::new();
    for id in ids {
        match cam_generate::job_for(state, &doc, &id) {
            Ok(job) => jobs.push(job),
            Err(why) => refused.push((id, why)),
        }
    }
    state.cam.job_builder.get_or_insert(cam_generate::job_at_dispatch);
    let queued = state.cam_enqueue(jobs);
    Ok((queued, refused))
}

/// Arm the reference picker for setup `id`'s field `key`, seeded with the
/// field's current names; refused for a key that is not a reference field.
pub fn begin_setup_pick(state: &mut EngineState, id: &str, key: &str) -> Result<(), String> {
    let doc = cam_document(state)?;
    let setup = doc.setup(id).ok_or_else(|| format!("no setup `{id}`"))?;
    let (path, filter, multiple, label) = cam_setup::pick_path(id, key).ok_or_else(|| format!("`{key}` is not a reference field of a setup"))?;
    let params = cam_setup::setup_form_params(setup, &doc.orientation(id));
    let seed: Vec<String> = match params.get(key) {
        Some(Value::Array(list)) => list.iter().filter_map(Value::as_str).map(str::to_string).collect(),
        Some(Value::String(s)) if !s.is_empty() => vec![s.clone()],
        _ => Vec::new(),
    };
    if state.ref_select_active() {
        state.finish_ref_select();
    }
    state.begin_ref_select_for_cam(id, path, format!("{}: {label}", setup.name), filter, multiple, seed);
    Ok(())
}

/// Read, change and write the block in one step; a block that cannot be read
/// refuses the change with the read error, and a refused change writes
/// nothing.
pub fn edit_cam_document<T>(
    state: &mut EngineState,
    coalesce: Option<&str>,
    change: impl FnOnce(&mut CamDocument) -> Result<T, String>,
) -> Result<T, String> {
    let mut doc = cam_document(state)?;
    let out = change(&mut doc)?;
    write_cam_document(state, &doc, coalesce);
    Ok(out)
}

/// Add a setup: returns the id.
pub fn add_setup(state: &mut EngineState, name: Option<&str>) -> Result<String, String> {
    edit_cam_document(state, None, |doc| Ok(doc.add_setup(name)))
}

/// Add a tool: returns the id.
pub fn add_tool(state: &mut EngineState, name: Option<&str>) -> Result<String, String> {
    edit_cam_document(state, None, |doc| Ok(doc.add_tool(name)))
}

/// Add an operation of strategy kind `kind` to `setup` with `tool`; refused,
/// naming what is missing, when any of them does not exist. Returns the id.
pub fn add_operation(state: &mut EngineState, setup: &str, tool: &str, kind: &str, name: Option<&str>) -> Result<String, String> {
    edit_cam_document(state, None, |doc| doc.add_operation(setup, tool, kind, name))
}

/// Add an NC program over `operations` (each must exist). Returns the id.
pub fn add_program(state: &mut EngineState, name: Option<&str>, post: &str, operations: &[String]) -> Result<String, String> {
    add_program_with(state, name, post, operations, &BTreeMap::new())
}

/// [`add_program`] with post properties, in one step: a refused property
/// leaves no program behind.
pub fn add_program_with(state: &mut EngineState, name: Option<&str>, post: &str, operations: &[String], properties: &BTreeMap<String, Value>) -> Result<String, String> {
    edit_cam_document(state, None, |doc| {
        let id = doc.add_program(name, post, operations)?;
        if !properties.is_empty() {
            doc.set_program_properties(&id, properties)?;
        }
        Ok(id)
    })
}

/// Set or clear (`null`) post properties of NC program `id`.
pub fn set_program_properties(state: &mut EngineState, id: &str, patch: &BTreeMap<String, Value>) -> Result<(), String> {
    edit_cam_document(state, None, |doc| doc.set_program_properties(id, patch))
}

/// Remove the record with `id` (see [`CamDocument::remove`]).
pub fn remove_record(state: &mut EngineState, id: &str) -> Result<(), String> {
    edit_cam_document(state, None, |doc| doc.remove(id))
}

/// One setup's resolved geometry, as the viewport draws it and `cam_state`
/// reports it.
#[derive(Clone, Debug, PartialEq)]
pub struct SetupGeometry {
    pub frame: Result<SetupFrame, String>,
    pub stock: Option<(Point3, Point3)>,
}

/// A stamp of everything the setups' geometry is read from: the document
/// revision (the block) and the scene's solids (count, content revisions,
/// visibility). Equal stamps mean equal frames and stock boxes.
fn geometry_stamp(state: &EngineState) -> (u64, u64) {
    let mut scene: u64 = 0xcbf2_9ce4_8422_2325;
    for solid in state.scene.solids() {
        for byte in solid.name.bytes().chain([solid.visible as u8]).chain(solid.revision.to_le_bytes()) {
            scene ^= byte as u64;
            scene = scene.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    (state.history.revision(), scene)
}

/// The block as the `cam_state` command and the `__brepCam` blob report it:
/// every record in its model serde form, each setup with the ids of its
/// operations, the strategy kinds an operation can be, the pane's selection
/// and its last notice — or, for a block that cannot be read, `error` and
/// empty lists. The setups' frames and stock boxes come from the pane's
/// cache ([`CamPanel::refresh_geometry`]), which the caller refreshes first.
pub fn cam_state_json(state: &EngineState, panel: &CamPanel) -> String {
    let (doc, error) = match cam_document(state) {
        Ok(doc) => (doc, None),
        Err(why) => (CamDocument::default(), Some(why)),
    };
    json!({
        "setups": doc.setups.iter().map(|s| {
            let mut record = serde_json::to_value(s).unwrap_or(Value::Null);
            if let Some(object) = record.as_object_mut() {
                object.insert("operations".into(), json!(doc.operations_of(s.id.as_str()).map(|op| op.id.as_str()).collect::<Vec<_>>()));
            }
            record
        }).collect::<Vec<_>>(),
        "tools": doc.tools,
        "operations": doc.operations,
        // What each operation's strategy still needs before it generates.
        "problems": doc.operations.iter().filter_map(|op| cam_operation::strategy_problem(op).map(|p| (op.id.as_str().to_string(), json!(p)))).collect::<serde_json::Map<String, Value>>(),
        "ncPrograms": doc.nc_programs,
        "multiAxisSources": doc.multi_axis_sources,
        "idCounter": doc.id_counter,
        "strategies": strategy_kinds(),
        "orientations": doc.orientations,
        // Every setup's resolved frame and stock box, as the viewport draws
        // them: `null` where the picks do not resolve (`frameError` says why).
        "frames": doc.setups.iter().map(|s| {
            let id = s.id.as_str();
            let geometry = panel.geometry.get(id).cloned().unwrap_or_else(|| SetupGeometry { frame: setup_frame(state, &doc, id), stock: cam_setup::stock_bounds(s, &state.scene) });
            let (frame, frame_error) = match geometry.frame {
                Ok(frame) => (Some(frame), None),
                Err(why) => (None, Some(why)),
            };
            let p = |p: Point3| json!([p.x, p.y, p.z]);
            (id.to_string(), json!({
                "origin": frame.map(|f| p(f.origin)),
                "xAxis": frame.map(|f| p(f.x_axis)),
                "zAxis": frame.map(|f| p(f.z_axis)),
                "frameError": frame_error,
                "stock": geometry.stock.map(|(min, max)| json!({ "min": p(min), "max": p(max) })),
                // A bounds-offset stock in the setup's own frame: top / bottom along machine Z.
                "stockMachine": cam_setup::stock_bounds_machine(s, &state.scene).map(|b| json!({ "min": p(b.min), "max": p(b.max) })),
            }))
        }).collect::<serde_json::Map<String, Value>>(),
        "selected": panel.selected,
        "open": panel.open,
        "notice": panel.notice,
        "error": error,
        // Generation: the engine's queue and results, and each operation's
        // status against the document as it stands (stale when its inputs moved).
        "generation": state.cam_generation_json(),
        "autoprogram": panel.autoprogram.state(state),
        "statuses": doc.operations.iter().map(|op| {
            let id = op.id.as_str();
            let status = cam_generate::status_of(state, &doc, id);
            let detail = match &status {
                cam_generate::OpStatus::Ok { motions, cutting_length } => json!({ "motions": motions, "cuttingLength": cutting_length }),
                cam_generate::OpStatus::Stale { motions } => json!({ "motions": motions }),
                cam_generate::OpStatus::Error(e) => json!({ "error": e }),
                _ => json!({}),
            };
            (id.to_string(), json!({ "status": status.word(), "detail": detail }))
        }).collect::<serde_json::Map<String, Value>>(),
        "scrub": panel.scrub,
        // The document's custom machines and the built-in library's ids (C17c).
        "machines": doc.machines,
        "builtinMachines": brep_cam::kinematics::library::builtin_ids(),
        "hiddenToolpaths": panel.hidden_toolpaths.iter().collect::<Vec<_>>(),
    })
    .to_string()
}

/// What the pane asks the shell for, drained once per frame by `app.rs`:
/// the file chooser (an import) and the save door (an export) are the
/// shell's, like the PLM pane's.
#[derive(Debug, Default)]
pub struct CamShellRequests {
    /// Open the file chooser for `(tag, title)`.
    pub pick_file: Option<(String, String)>,
    /// Files to save or download.
    pub save_files: Vec<SaveRequest>,
}

/// The chooser tag a tool import is asked under.
pub const IMPORT_TOOLS_TAG: &str = "cam-tools";

/// The pane's transient UI state.
pub struct CamPanel {
    pub autoprogram: crate::panels::cam_autoprogram::AutoProgramPanel,
    hits: HashMap<String, egui::Rect>,
    /// The selected row's id (a setup, operation, tool or program).
    selected: Option<String>,
    /// An action handed over by a toolbar button, performed on the next draw.
    request: Option<CamRequest>,
    /// What the last action said that was not a record (a refusal, a count),
    /// drawn under the header until the next action.
    notice: Option<String>,
    /// The setup whose dialog is open, drawn in place of the tree.
    open: Option<String>,
    /// The form field (its key) a pick is running in, so the form draws that
    /// field focused: the engine's pick state carries the BLOCK path, which
    /// is not the field's.
    picking_field: Option<String>,
    /// The last overlay group pushed, so an unchanged frame costs nothing.
    overlay: String,
    /// Every setup's resolved frame and stock box, by setup id, as of
    /// `geometry_stamp` — walking every visible solid's mesh is O(mesh), and
    /// the state blob is published every frame, so it is done once per
    /// change, not once per frame.
    geometry: HashMap<String, SetupGeometry>,
    geometry_stamp: Option<(u64, u64)>,
    /// Requests for the shell, taken each frame ([`Self::take_shell_requests`]).
    shell: CamShellRequests,
    /// The user library's tools (id, name, number), re-read when the pane
    /// draws the library menu; `None` until then.
    user_library: Option<Result<Vec<(String, String, u32)>, String>>,
    /// The last import's outcome: ids added and rows skipped.
    import_report: Option<(Vec<String>, Vec<String>)>,
    /// The work material chosen in the tool dialog for Calculate feeds —
    /// the calculator's input, not a tool field, so it lives here.
    feeds_material: Option<String>,
    /// The toolpath preview's scrub position on the selected operation, as a
    /// fraction of its segments; `None` draws no tool marker.
    scrub: Option<f64>,
    /// The simulation player section (lane C17), drawn under the tree.
    pub sim: crate::panels::cam_sim::CamSimPanel,
    /// The NC program post preview (contract C19-27), drawn under the tree.
    pub post: crate::panels::cam_post::CamPostPanel,
    /// The toolpath overlay groups last pushed, by operation, so a result
    /// that is gone is removed from the viewport.
    overlay_ops: std::collections::BTreeSet<String>,
    /// What the pushed overlay was built from (the setup group's text, the
    /// results stamp, the selection, the scrub, the hidden set's stamp), so a
    /// frame with nothing changed serialises no toolpath.
    overlay_key: Option<(String, u64, Option<String>, Option<u64>, u64)>,
    /// Operations whose toolpath is hidden in the viewport (the row menu's
    /// Hide toolpath / `cam_toolpath_visible`); pane state, not the document's.
    hidden_toolpaths: std::collections::BTreeSet<String>,
    hidden_stamp: u64,
}

impl Default for CamPanel {
    fn default() -> Self {
        Self::new()
    }
}

/// Section hit keys.
const SECTION_SETUPS: &str = "cam:section:setups";
const SECTION_PROGRAMS: &str = "cam:section:programs";
const SECTION_TOOLS: &str = "cam:section:tools";
const SECTION_MACHINES: &str = "cam:section:machines";

impl CamPanel {
    pub fn new() -> Self {
        Self {
            autoprogram: Default::default(),
            hits: HashMap::new(),
            selected: None,
            request: None,
            notice: None,
            open: None,
            picking_field: None,
            overlay: String::new(),
            geometry: HashMap::new(),
            geometry_stamp: None,
            shell: CamShellRequests::default(),
            user_library: None,
            import_report: None,
            feeds_material: None,
            scrub: None,
            sim: crate::panels::cam_sim::CamSimPanel::new(),
            post: crate::panels::cam_post::CamPostPanel::new(),
            overlay_ops: std::collections::BTreeSet::new(),
            overlay_key: None,
            hidden_toolpaths: std::collections::BTreeSet::new(),
            hidden_stamp: 0,
        }
    }

    /// Show or hide operation `id`'s toolpath in the viewport.
    pub fn set_toolpath_visible(&mut self, id: &str, visible: bool) {
        let changed = if visible { self.hidden_toolpaths.remove(id) } else { self.hidden_toolpaths.insert(id.to_string()) };
        if changed {
            self.hidden_stamp += 1;
        }
    }

    /// The operations whose toolpath is hidden.
    pub fn hidden_toolpaths(&self) -> &std::collections::BTreeSet<String> {
        &self.hidden_toolpaths
    }

    /// Set the preview's scrub fraction (`None` clears the marker).
    pub fn set_scrub(&mut self, fraction: Option<f64>) {
        self.scrub = fraction.map(|f| f.clamp(0.0, 1.0));
    }

    /// The requests the pane made this frame, for the shell.
    pub fn take_shell_requests(&mut self) -> CamShellRequests {
        if let Some(save) = self.sim.pending_save.take() {
            self.shell.save_files.push(save);
        }
        if let Some(save) = self.post.pending_save.take() {
            self.shell.save_files.push(save);
        }
        std::mem::take(&mut self.shell)
    }

    /// The file the chooser delivered for an import: read as text, imported
    /// into the document's tools, the outcome shown under the header.
    pub fn deliver_picked(&mut self, state: &mut EngineState, name: &str, bytes: &[u8]) {
        let text = String::from_utf8_lossy(bytes);
        match import_tools(state, name, &text) {
            Ok((ids, issues)) => {
                self.notice = Some(format!(
                    "imported {} tool{} from {name}{}",
                    ids.len(),
                    if ids.len() == 1 { "" } else { "s" },
                    if issues.is_empty() { String::new() } else { format!(" ({} row{} skipped)", issues.len(), if issues.len() == 1 { "" } else { "s" }) }
                ));
                self.import_report = Some((ids, issues));
            }
            Err(why) => self.refuse(state, format!("import {name}: {why}")),
        }
    }

    /// The user library as the pane last read it (`None` = not read yet).
    pub fn user_library(&self) -> Option<&Result<Vec<(String, String, u32)>, String>> {
        self.user_library.as_ref()
    }

    /// Re-read the user library's listing from the store.
    pub fn refresh_user_library(&mut self, store: &dyn ModelStore) {
        self.user_library = Some(cam_library::load(store).map(|lib| lib.library.tools.iter().map(|t| (t.id.as_str().to_string(), t.name.clone(), t.number)).collect()));
    }

    /// Bring the setups' geometry cache up to date with the document and the
    /// scene. Cheap when nothing changed (two counters and a walk of the
    /// solid list's headers); the callers that publish or draw from the
    /// cache call it first.
    pub fn refresh_geometry(&mut self, state: &EngineState) {
        let stamp = geometry_stamp(state);
        if self.geometry_stamp == Some(stamp) {
            return;
        }
        self.geometry.clear();
        if let Ok(doc) = cam_document(state) {
            for setup in &doc.setups {
                let id = setup.id.as_str();
                self.geometry.insert(
                    id.to_string(),
                    SetupGeometry { frame: setup_frame(state, &doc, id), stock: cam_setup::stock_bounds(setup, &state.scene) },
                );
            }
        }
        self.geometry_stamp = Some(stamp);
    }

    /// The setup whose dialog is open.
    pub fn open_setup(&self) -> Option<&str> {
        self.open.as_deref()
    }

    /// Open setup `id`'s dialog (`None` closes it).
    pub fn set_open(&mut self, id: Option<String>) {
        if id != self.open {
            self.picking_field = None;
        }
        self.open = id;
    }

    /// Hand the pane an action; it runs on the next frame the pane draws.
    pub fn request(&mut self, request: CamRequest) {
        self.request = Some(request);
    }

    /// The selected row's id.
    pub fn selected(&self) -> Option<&str> {
        self.selected.as_deref()
    }

    /// Select a row by id (`None` clears). The id is not checked against the
    /// block here: a row that no longer exists simply draws nothing selected.
    pub fn select(&mut self, id: Option<String>) {
        self.selected = id;
    }

    /// The setup an operation would go into: the selected setup, the selected
    /// operation's setup, or the only setup there is.
    fn target_setup(&self, doc: &CamDocument) -> Option<String> {
        if let Some(selected) = &self.selected {
            if doc.setup(selected).is_some() {
                return Some(selected.clone());
            }
            if let Some(op) = doc.operation(selected) {
                return Some(op.setup.as_str().to_string());
            }
        }
        match doc.setups.as_slice() {
            [only] => Some(only.id.as_str().to_string()),
            _ => None,
        }
    }

    /// The tool an operation would use: the selected tool, or the only one.
    fn target_tool(&self, doc: &CamDocument) -> Option<String> {
        if let Some(selected) = &self.selected {
            if doc.tool(selected).is_some() {
                return Some(selected.clone());
            }
        }
        match doc.tools.as_slice() {
            [only] => Some(only.id.as_str().to_string()),
            _ => None,
        }
    }

    /// Say why an action did not produce a record: a toast, and the line
    /// under the header.
    fn refuse(&mut self, state: &mut EngineState, why: String) {
        state.push_notice(format!("CAM: {why}"));
        self.notice = Some(why);
    }

    /// Perform a handed-over action against the engine.
    fn perform(&mut self, request: CamRequest, state: &mut EngineState) {
        self.notice = None;
        let doc = match cam_document(state) {
            Ok(doc) => doc,
            Err(why) => return self.refuse(state, why),
        };
        let outcome = match request {
            CamRequest::NewSetup => add_setup(state, None),
            CamRequest::NewTool => add_tool(state, None),
            CamRequest::NewOperation => match (self.target_setup(&doc), self.target_tool(&doc)) {
                (Some(setup), Some(tool)) => add_operation(state, &setup, &tool, "parallel", None),
                (None, _) if doc.setups.is_empty() => Err("no setup yet: press New setup first".to_string()),
                (None, _) => Err("select the setup the operation goes into".to_string()),
                (_, None) if doc.tools.is_empty() => Err("no tool yet: press New tool first".to_string()),
                (_, None) => Err("select the tool the operation uses".to_string()),
            },
            CamRequest::Generate => {
                if doc.operations.is_empty() {
                    Err("nothing to generate: add an operation first".to_string())
                } else {
                    match generate(state, None) {
                        Ok((queued, refused)) => {
                            if !refused.is_empty() {
                                self.notice = Some(refused.iter().map(|(_, why)| why.as_str()).collect::<Vec<_>>().join("; "));
                            }
                            if queued.is_empty() && refused.is_empty() {
                                self.notice = Some("every operation is already queued or generating".to_string());
                            }
                            return;
                        }
                        Err(why) => Err(why),
                    }
                }
            }
        };
        match outcome {
            Ok(id) => self.selected = Some(id),
            Err(why) => self.refuse(state, why),
        }
    }

    /// Draw the pane: perform any handed-over action, then the header and the
    /// three sections, applying at most one deferred engine mutation.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, store: &dyn ModelStore) {
        self.hits.clear();
        // Unprefixed, as `scroll_into_view` looks it up (`<panel>/panel:clip`).
        self.hits.insert("panel:clip".into(), ui.clip_rect());
        if let Some(request) = self.request.take() {
            self.perform(request, state);
        }
        self.refresh_geometry(state);
        let (doc, error) = match cam_document(state) {
            Ok(doc) => (doc, None),
            Err(why) => (CamDocument::default(), Some(why)),
        };

        self.autoprogram.poll(state);
        // --- header -----------------------------------------------------------
        let mut request = None;
        ui.horizontal_wrapped(|ui| {
            for (key, label, hover, what) in [
                ("cam:new:setup", "New setup", "Add a setup (work coordinate system, stock, heights)", CamRequest::NewSetup),
                ("cam:new:tool", "New tool", "Add a tool to the document's library", CamRequest::NewTool),
                ("cam:new:operation", "New operation", "Add an operation to the selected setup with the selected tool", CamRequest::NewOperation),
                ("cam:generate", "Generate", "Compute the toolpaths of every operation that needs it", CamRequest::Generate),
            ] {
                let button = ui.button(label).on_hover_text(hover);
                self.hits.insert(key.into(), button.rect);
                if button.clicked() {
                    request = Some(what);
                }
            }
            let auto = ui.button("Program part automatically");
            self.hits.insert("cam:auto:open".into(), auto.rect);
            if auto.clicked() {
                self.open = None;
                let setup = self.target_setup(&doc);
                match setup.ok_or_else(|| "select a setup before programming".to_string()).and_then(|id| self.autoprogram.preview(state, &id)) { Ok(()) => {}, Err(why) => self.notice = Some(why) }
            }
            ui.add(
                egui::Label::new(
                    egui::RichText::new(format!(
                        "{} setup{} | {} tool{} | {} operation{}",
                        doc.setups.len(),
                        if doc.setups.len() == 1 { "" } else { "s" },
                        doc.tools.len(),
                        if doc.tools.len() == 1 { "" } else { "s" },
                        doc.operations.len(),
                        if doc.operations.len() == 1 { "" } else { "s" },
                    ))
                    .weak(),
                )
                .wrap_mode(egui::TextWrapMode::Extend),
            );
        });
        if let Some(error) = &error {
            let label = ui.label(egui::RichText::new(error).color(egui::Color32::from_rgb(0xf8, 0x51, 0x49)));
            self.hits.insert("cam:error".into(), label.rect);
        }
        if let Some(notice) = &self.notice {
            let label = ui.label(egui::RichText::new(notice).weak().color(egui::Color32::from_rgb(0xd2, 0x99, 0x22)));
            self.hits.insert("cam:notice".into(), label.rect);
        }
        if state.cam.busy() {
            ui.horizontal_wrapped(|ui| {
                let text = match (&state.cam.progress, state.cam.in_flight()) {
                    (Some(p), _) if p.total > 0 => format!("generating {}: {} {}/{}", p.operation, p.phase, p.current, p.total),
                    (Some(p), _) => format!("generating {}: {}", p.operation, p.phase),
                    (None, Some(op)) => format!("generating {op}\u{2026}"),
                    (None, None) => "generation queued\u{2026}".to_string(),
                };
                let queued = state.cam.queued().len();
                let line = ui.label(egui::RichText::new(if queued > 0 { format!("{text} (+{queued} queued)") } else { text }).weak());
                self.hits.insert("cam:progress".into(), line.rect);
                let cancel = ui.button("Cancel").on_hover_text("Drop the queue; the operation generating stops at its next chunk where the runner can");
                self.hits.insert("cam:cancel".into(), cancel.rect);
                if cancel.clicked() {
                    state.cam_cancel();
                }
            });
        } else if let Some(notice) = &state.cam.notice {
            let line = ui.label(egui::RichText::new(notice).weak());
            self.hits.insert("cam:progress".into(), line.rect);
        }
        ui.add_space(2.0);

        if self.autoprogram.open {
            self.autoprogram.show(ui, state, &mut self.hits);
            if let Some(request) = request {
                self.perform(request, state);
            }
            return;
        }

        // --- the open setup's dialog, in place of the tree ----------------------
        if let Some(id) = self.open.clone() {
            if doc.setup(&id).is_some() {
                self.show_setup_form(ui, state, &doc, &id);
                if let Some(request) = request {
                    self.perform(request, state);
                }
                return;
            }
            if doc.tool(&id).is_some() {
                self.show_tool_form(ui, state, store, &doc, &id);
                if let Some(request) = request {
                    self.perform(request, state);
                }
                return;
            }
            if doc.operation(&id).is_some() {
                self.show_operation_form(ui, state, &doc, &id);
                if let Some(request) = request {
                    self.perform(request, state);
                }
                return;
            }
            if doc.machines.iter().any(|m| m.id.as_str() == id) {
                self.show_machine_form(ui, state, &doc, &id);
                if let Some(request) = request {
                    self.perform(request, state);
                }
                return;
            }
            self.open = None;
        }
        self.push_overlay(state, &doc, self.selected.clone());

        // --- the tree ---------------------------------------------------------
        let mut clicked: Option<String> = None;
        let mut remove: Option<String> = None;
        let mut open_operation: Option<String> = None;
        // (operation id, currently hidden) from the row menu's Hide / Show toolpath.
        let mut toggle_toolpath: Option<(String, bool)> = None;
        let row = |ui: &mut egui::Ui, hits: &mut HashMap<String, egui::Rect>, id: &str, text: String, selected: bool, clicked: &mut Option<String>, remove: &mut Option<String>| {
            ui.horizontal(|ui| {
                let label = ui.selectable_label(selected, text);
                hits.insert(format!("cam:row:{id}"), label.rect);
                if label.clicked() {
                    *clicked = Some(id.to_string());
                }
                label.context_menu(|ui| {
                    let delete = ui.button("Delete");
                    hits.insert(format!("cam:menuitem:{id}:delete"), delete.rect);
                    if delete.clicked() {
                        *remove = Some(id.to_string());
                        ui.close();
                    }
                });
            });
        };

        let setups = egui::CollapsingHeader::new(format!("Setups ({})", doc.setups.len()))
            .id_salt("cam-setups")
            .default_open(true)
            .show(ui, |ui| {
                if doc.setups.is_empty() {
                    ui.label(egui::RichText::new("No setups \u{2014} press New setup").weak());
                }
                for setup in &doc.setups {
                    let id = setup.id.as_str();
                    let ops: Vec<_> = doc.operations_of(id).collect();
                    let selected = self.selected.as_deref() == Some(id);
                    let header = egui::CollapsingHeader::new(setup.name.as_str())
                        .id_salt(("cam-setup", id))
                        .default_open(true)
                        .show(ui, |ui| {
                            if ops.is_empty() {
                                ui.label(egui::RichText::new("No operations \u{2014} press New operation").weak());
                            }
                            for op in ops {
                                let op_id = op.id.as_str();
                                let status = cam_generate::status_of(state, &doc, op_id);
                                let text = format!("{} {}  \u{00b7} {} \u{00b7} {}{}", status.mark(), op.name, op.strategy.kind_name(), op.tool, if op.enabled { "" } else { " \u{00b7} off" });
                                ui.horizontal(|ui| {
                                    let hover = match &status {
                                        cam_generate::OpStatus::Ok { motions, cutting_length } => format!("generated: {motions} motions, {cutting_length:.2} cutting"),
                                        cam_generate::OpStatus::Stale { motions } => format!("stale: {motions} motions from inputs that have changed; generate again"),
                                        cam_generate::OpStatus::Error(e) => format!("failed: {e}"),
                                        other => other.word().to_string(),
                                    };
                                    let label = ui.selectable_label(self.selected.as_deref() == Some(op_id), text).on_hover_text(hover);
                                    self.hits.insert(format!("cam:row:{op_id}"), label.rect);
                                    if label.double_clicked() {
                                        open_operation = Some(op_id.to_string());
                                    } else if label.clicked() {
                                        clicked = Some(op_id.to_string());
                                    }
                                    label.context_menu(|ui| {
                                        let edit = ui.button("Edit operation\u{2026}");
                                        self.hits.insert(format!("cam:menuitem:{op_id}:edit"), edit.rect);
                                        if edit.clicked() {
                                            open_operation = Some(op_id.to_string());
                                            ui.close();
                                        }
                                        let hidden = self.hidden_toolpaths.contains(op_id);
                                        let toggle = ui.button(if hidden { "Show toolpath" } else { "Hide toolpath" });
                                        self.hits.insert(format!("cam:menuitem:{op_id}:toolpath"), toggle.rect);
                                        if toggle.clicked() {
                                            toggle_toolpath = Some((op_id.to_string(), hidden));
                                            ui.close();
                                        }
                                        let delete = ui.button("Delete");
                                        self.hits.insert(format!("cam:menuitem:{op_id}:delete"), delete.rect);
                                        if delete.clicked() {
                                            remove = Some(op_id.to_string());
                                            ui.close();
                                        }
                                    });
                                });
                            }
                        });
                    self.hits.insert(format!("cam:row:{id}"), header.header_response.rect);
                    if header.header_response.double_clicked() {
                        self.open = Some(id.to_string());
                        self.selected = Some(id.to_string());
                    } else if header.header_response.clicked() {
                        clicked = Some(id.to_string());
                    }
                    header.header_response.context_menu(|ui| {
                        let edit = ui.button("Edit setup\u{2026}");
                        self.hits.insert(format!("cam:menuitem:{id}:edit"), edit.rect);
                        if edit.clicked() {
                            self.open = Some(id.to_string());
                            ui.close();
                        }
                    });
                    header.header_response.context_menu(|ui| {
                        let delete = ui.button("Delete setup and its operations");
                        self.hits.insert(format!("cam:menuitem:{id}:delete"), delete.rect);
                        if delete.clicked() {
                            remove = Some(id.to_string());
                            ui.close();
                        }
                    });
                    if selected {
                        ui.painter().rect_stroke(
                            header.header_response.rect,
                            2.0,
                            egui::Stroke::new(1.0, ui.visuals().selection.stroke.color),
                            egui::StrokeKind::Inside,
                        );
                    }
                }
            });
        self.hits.insert(SECTION_SETUPS.into(), setups.header_response.rect);

        let mut post_program: Option<String> = None;
        let programs = egui::CollapsingHeader::new(format!("NC Programs ({})", doc.nc_programs.len()))
            .id_salt("cam-programs")
            .default_open(true)
            .show(ui, |ui| {
                if doc.nc_programs.is_empty() {
                    ui.label(egui::RichText::new("No NC programs \u{2014} add one, then Post\u{2026} it to an NC file").weak());
                }
                for program in &doc.nc_programs {
                    let text = format!("{}  \u{00b7} {} \u{00b7} {} op{}", program.name, program.post, program.operations.len(), if program.operations.len() == 1 { "" } else { "s" });
                    ui.horizontal(|ui| {
                        let b = ui.small_button("Post\u{2026}").on_hover_text("Post through the program's post: preview the NC text and save it");
                        self.hits.insert(format!("cam:post:program:{}", program.id), b.rect);
                        if b.clicked() {
                            post_program = Some(program.id.clone());
                        }
                        row(ui, &mut self.hits, &program.id, text, self.selected.as_deref() == Some(program.id.as_str()), &mut clicked, &mut remove);
                    });
                }
            });
        self.hits.insert(SECTION_PROGRAMS.into(), programs.header_response.rect);
        if let Some(id) = post_program {
            let _ = self.post.open(state, &doc, &id);
        }

        let mut open_tool: Option<String> = None;
        let mut duplicate: Option<String> = None;
        let mut to_library: Option<String> = None;
        let mut from_library: Option<String> = None;
        let mut import = false;
        let mut export: Option<&'static str> = None;
        let tools = egui::CollapsingHeader::new(format!("Tool Library ({})", doc.tools.len()))
            .id_salt("cam-tools")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    let add = ui.menu_button("Add from user library", |ui| {
                        if self.user_library.is_none() {
                            self.refresh_user_library(store);
                        }
                        match self.user_library.as_ref() {
                            Some(Ok(list)) if list.is_empty() => {
                                ui.label(egui::RichText::new("The user library is empty \u{2014} save a tool to it from its row menu").weak());
                            }
                            Some(Ok(list)) => {
                                for (id, name, number) in list.clone() {
                                    let entry = ui.button(format!("T{number}  {name}"));
                                    self.hits.insert(format!("cam:library:{id}"), entry.rect);
                                    if entry.clicked() {
                                        from_library = Some(id);
                                        ui.close();
                                    }
                                }
                            }
                            Some(Err(why)) => {
                                ui.label(egui::RichText::new(why).color(egui::Color32::from_rgb(0xf8, 0x51, 0x49)));
                            }
                            None => {}
                        }
                    });
                    self.hits.insert("cam:tools:library".into(), add.response.rect);
                    if add.response.clicked() {
                        self.refresh_user_library(store);
                    }
                    let import_button = ui.button("Import\u{2026}").on_hover_text("Add the tools of a Fusion 360 export (.json / .tools), a .csv sheet or a library file");
                    self.hits.insert("cam:tools:import".into(), import_button.rect);
                    import = import_button.clicked();
                    let export_doc = ui.button("Export").on_hover_text("Save the document's tools as a library file");
                    self.hits.insert("cam:tools:export".into(), export_doc.rect);
                    if export_doc.clicked() {
                        export = Some("document");
                    }
                    let export_user = ui.button("Export user library").on_hover_text("Save the user library as a library file");
                    self.hits.insert("cam:tools:export-user".into(), export_user.rect);
                    if export_user.clicked() {
                        export = Some("user");
                    }
                });
                if let Some((ids, issues)) = &self.import_report {
                    for issue in issues {
                        ui.label(egui::RichText::new(format!("skipped {issue}")).weak().color(egui::Color32::from_rgb(0xd2, 0x99, 0x22)));
                    }
                    let _ = ids;
                }
                if doc.tools.is_empty() {
                    ui.label(egui::RichText::new("No tools \u{2014} press New tool").weak());
                }
                for tool in &doc.tools {
                    let id = tool.id.as_str();
                    let text = format!("T{}  {} \u{00b7} {} \u{2300}{}", tool.number, tool.name, tool.kind.as_str(), tool.geometry.diameter);
                    ui.horizontal(|ui| {
                        let label = ui.selectable_label(self.selected.as_deref() == Some(id), text);
                        self.hits.insert(format!("cam:row:{id}"), label.rect);
                        if label.double_clicked() {
                            open_tool = Some(id.to_string());
                        } else if label.clicked() {
                            clicked = Some(id.to_string());
                        }
                        label.context_menu(|ui| {
                            let edit = ui.button("Edit tool\u{2026}");
                            self.hits.insert(format!("cam:menuitem:{id}:edit"), edit.rect);
                            if edit.clicked() {
                                open_tool = Some(id.to_string());
                                ui.close();
                            }
                            let dup = ui.button("Duplicate");
                            self.hits.insert(format!("cam:menuitem:{id}:duplicate"), dup.rect);
                            if dup.clicked() {
                                duplicate = Some(id.to_string());
                                ui.close();
                            }
                            let save = ui.button("Save to user library");
                            self.hits.insert(format!("cam:menuitem:{id}:library"), save.rect);
                            if save.clicked() {
                                to_library = Some(id.to_string());
                                ui.close();
                            }
                            let delete = ui.button("Delete");
                            self.hits.insert(format!("cam:menuitem:{id}:delete"), delete.rect);
                            if delete.clicked() {
                                remove = Some(id.to_string());
                                ui.close();
                            }
                        });
                    });
                }
            });
        self.hits.insert(SECTION_TOOLS.into(), tools.header_response.rect);

        // --- Machines: the document's own machines, each a copy of a built-in --
        let mut open_machine: Option<String> = None;
        let mut new_machine: Option<String> = None;
        let machines = egui::CollapsingHeader::new(format!("Machines ({})", doc.machines.len()))
            .id_salt("cam-machines")
            .default_open(true)
            .show(ui, |ui| {
                let add = ui.menu_button("New machine", |ui| {
                    for base in brep_cam::kinematics::library::builtin_machines() {
                        let id = base.id.as_str().to_string();
                        let entry = ui.button(format!("{}  \u{00b7} {}", base.name, cam_machine::kind_tag(&base.kind)));
                        self.hits.insert(format!("cam:machines:new:{id}"), entry.rect);
                        if entry.clicked() {
                            new_machine = Some(id);
                            ui.close();
                        }
                    }
                });
                self.hits.insert("cam:machines:new".into(), add.response.rect);
                if doc.machines.is_empty() {
                    ui.label(egui::RichText::new("No machines of this document \u{2014} a setup can name a built-in directly").weak());
                }
                for machine in &doc.machines {
                    let id = machine.id.as_str();
                    let used = doc.setups.iter().filter(|s| s.machine.as_ref().is_some_and(|m| m.as_str() == id)).count();
                    let text = format!(
                        "{}  \u{00b7} {}{}{}",
                        machine.name,
                        cam_machine::kind_tag(&machine.kind),
                        machine.post.as_deref().map(|p| format!(" \u{00b7} {p}")).unwrap_or_default(),
                        if used > 0 { format!(" \u{00b7} {used} setup{}", if used == 1 { "" } else { "s" }) } else { String::new() }
                    );
                    let label = ui.selectable_label(self.selected.as_deref() == Some(id), text);
                    self.hits.insert(format!("cam:row:{id}"), label.rect);
                    if label.double_clicked() {
                        open_machine = Some(id.to_string());
                    } else if label.clicked() {
                        clicked = Some(id.to_string());
                    }
                    label.context_menu(|ui| {
                        let edit = ui.button("Edit machine\u{2026}");
                        self.hits.insert(format!("cam:menuitem:{id}:edit"), edit.rect);
                        if edit.clicked() {
                            open_machine = Some(id.to_string());
                            ui.close();
                        }
                        let delete = ui.button("Delete");
                        self.hits.insert(format!("cam:menuitem:{id}:delete"), delete.rect);
                        if delete.clicked() {
                            remove = Some(id.to_string());
                            ui.close();
                        }
                    });
                }
            });
        self.hits.insert(SECTION_MACHINES.into(), machines.header_response.rect);
        if let Some(base) = new_machine {
            match add_machine(state, &base, None) {
                Ok(id) => {
                    self.notice = None;
                    self.selected = Some(id);
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(id) = open_machine {
            self.selected = Some(id.clone());
            self.open = Some(id);
        }
        if let Some(id) = open_tool {
            self.selected = Some(id.clone());
            self.open = Some(id);
        }
        if let Some(id) = duplicate {
            match duplicate_tool(state, &id) {
                Ok(new_id) => self.selected = Some(new_id),
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(id) = to_library {
            let outcome = doc.tool(&id).ok_or_else(|| format!("no tool `{id}`")).and_then(|tool| cam_library::save_tool(store, tool));
            match outcome {
                Ok(()) => {
                    self.notice = Some(format!("{id} saved to the user library"));
                    self.user_library = None;
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(library_id) = from_library {
            match add_tool_from_library(state, store, &library_id) {
                Ok(new_id) => self.selected = Some(new_id),
                Err(why) => self.refuse(state, why),
            }
        }
        if import {
            self.shell.pick_file = Some((IMPORT_TOOLS_TAG.to_string(), "Import tools".to_string()));
        }
        if let Some(which) = export {
            let outcome = if which == "user" { cam_library::export_text(store) } else { export_document_tools(state) };
            match outcome {
                Ok(text) => self.shell.save_files.push(SaveRequest {
                    title: if which == "user" { "Export the user tool library".into() } else { "Export the document's tools".into() },
                    file_name: if which == "user" { "user-tools.json".into() } else { "tools.json".into() },
                    body: SaveBody::Text(text),
                }),
                Err(why) => self.refuse(state, why),
            }
        }

        // --- deferred mutations -----------------------------------------------
        if let Some(id) = open_operation {
            self.selected = Some(id.clone());
            self.open = Some(id);
        }
        if let Some(id) = clicked {
            self.selected = if self.selected.as_deref() == Some(id.as_str()) { None } else { Some(id) };
        }
        if let Some((id, hidden)) = toggle_toolpath {
            self.set_toolpath_visible(&id, hidden);
        }
        if let Some(id) = remove {
            match remove_record(state, &id) {
                Ok(()) => {
                    if self.selected.as_deref() == Some(id.as_str()) {
                        self.selected = None;
                    }
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(request) = request {
            self.perform(request, state);
        }
        // --- the simulation player (lane C17) ---------------------------------
        if error.is_none() {
            ui.separator();
            let setup_hint = self.selected.as_deref().and_then(|id| {
                doc.setup(id).map(|_| id.to_string()).or_else(|| doc.operation(id).map(|o| o.setup.to_string()))
            });
            self.sim.show(ui, state, &doc, setup_hint.as_deref());
        }
        // --- the NC program post preview (contract C19-27) ---------------------
        if let Some((id, patch)) = self.post.show(ui, &doc) {
            match set_program_properties(state, &id, &patch) {
                Ok(()) => match cam_document(state) {
                    Ok(doc) => {
                        let _ = self.post.open(state, &doc, &id);
                    }
                    Err(e) => self.post.error = Some(e),
                },
                Err(e) => self.post.error = Some(e),
            }
        }
    }

    /// Draw setup `id`'s dialog: the shared form over its params, its picks
    /// through the engine's CAM picker, the frame it resolves to written back,
    /// and the viewport overlay for this setup.
    fn show_setup_form(&mut self, ui: &mut egui::Ui, state: &mut EngineState, doc: &CamDocument, id: &str) {
        let Some(setup) = doc.setup(id) else { return };
        let orientation: SetupOrientation = doc.orientation(id);
        let fields = cam_setup::setup_form_fields();
        let mut params = cam_setup::setup_form_params(setup, &orientation);
        cam_setup::merge_machine_params(&mut params, doc, setup);
        let hidden = cam_setup::hidden_fields(&params);
        let title = format!("Setup {id}");
        let frame_error = self.geometry.get(id).and_then(|g| g.frame.clone().err());
        let banner = match (&frame_error, &self.notice) {
            (Some(why), _) => Some((why.as_str(), egui::Color32::from_rgb(0xf8, 0x51, 0x49))),
            (None, Some(notice)) => Some((notice.as_str(), egui::Color32::from_rgb(0xd2, 0x99, 0x22))),
            (None, None) => None,
        };
        let picking = state
            .ref_select_field()
            .filter(|f| f.target == RefSelectTarget::Cam && f.subject == id)
            .and_then(|f| self.picking_field.clone().map(|key| (vec![key], f.names)));
        let active = picking.as_ref().map(|(path, names)| crate::form::ActivePick { path, names });
        if active.is_some() {
            state.ref_select_claim();
        }
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(setup.name.as_str()),
            fields: &fields,
            hidden: Some(&hidden),
            banner,
            trailing: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "cam:",
            read_only: None,
            picking: active,
        };
        let was_picking = self.picking_field.is_some();
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("cam:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("cam:form:setup:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        self.push_overlay(state, doc, Some(id.to_string()));
        if let Some(activate) = out.ref_activate {
            let key = activate.path.first().cloned().unwrap_or_default();
            match begin_setup_pick(state, id, &key) {
                Ok(()) => {
                    self.picking_field = Some(key);
                    // The shell finishes an in-form pick nobody claimed this
                    // frame; the form draws it focused from the next one.
                    state.ref_select_claim();
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(index) = out.pick_remove {
            state.ref_select_remove(index);
        }
        if out.pick_done {
            state.finish_ref_select();
            self.picking_field = None;
            if let Some(why) = sync_setup_frame(state, id) {
                self.notice = Some(why);
            }
        }
        if out.changed {
            self.notice = None;
            let coalesce = format!("setup:{id}:form");
            if let Err(why) = update_setup(state, id, &params, Some(&coalesce)) {
                self.refuse(state, why);
            }
        }
        if out.exit_clicked {
            if state.ref_select_active() {
                state.finish_ref_select();
            }
            self.picking_field = None;
            self.open = None;
        }
        if was_picking && !state.ref_select_active() && self.picking_field.is_some() {
            // The pick ended outside the form (a single field finishes on its
            // first pick, by the shell): the frame follows it here.
            self.picking_field = None;
            if let Some(why) = sync_setup_frame(state, id) {
                self.notice = Some(why);
            }
        }
        if !state.ref_select_active() {
            self.picking_field = None;
        }
    }

    /// Draw operation `id`'s dialog: the generic form over its strategy and
    /// linking parameters plus the fixed groups; picks through the engine's
    /// CAM picker with the owning solid of a picked face filled from the
    /// scene; every edit one coalesced undo step.
    fn show_operation_form(&mut self, ui: &mut egui::Ui, state: &mut EngineState, doc: &CamDocument, id: &str) {
        let Some(operation) = doc.operation(id) else { return };
        let mut fields = cam_operation::operation_form_fields(operation, &doc.tools);
        fields.extend(crate::panels::cam_multiaxis::form_fields(doc, operation));
        let mut params = cam_operation::operation_form_params(operation);
        crate::panels::cam_multiaxis::merge_params(doc, id, &mut params);
        let hidden = cam_operation::hidden_fields(&params);
        let title = format!("Operation {id}");
        let subtitle = format!("{} \u{00b7} {} \u{00b7} {}", operation.strategy.kind_name(), operation.setup, operation.tool);
        let problem = cam_operation::strategy_problem(operation);
        let banner = match (&self.notice, &problem) {
            (Some(n), _) => Some((n.as_str(), egui::Color32::from_rgb(0xd2, 0x99, 0x22))),
            (None, Some(p)) => Some((p.as_str(), egui::Color32::from_rgb(0xd2, 0x99, 0x22))),
            (None, None) => None,
        };
        let picking = state
            .ref_select_field()
            .filter(|f| f.target == RefSelectTarget::Cam && f.subject == id)
            .and_then(|f| self.picking_field.clone().map(|key| (vec!["targets".to_string(), key], f.names)));
        let active = picking.as_ref().map(|(path, names)| crate::form::ActivePick { path, names });
        if active.is_some() {
            state.ref_select_claim();
        }
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(subtitle.as_str()),
            fields: &fields,
            hidden: Some(&hidden),
            banner,
            trailing: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "cam:",
            read_only: None,
            picking: active,
        };
        let was_picking = self.picking_field.is_some();
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("cam:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("cam:form:operation:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        self.push_overlay(state, doc, Some(operation.setup.as_str().to_string()));
        if let Some(activate) = out.ref_activate {
            let key = activate.path.last().cloned().unwrap_or_default();
            match begin_operation_pick(state, id, &key) {
                Ok(()) => {
                    self.picking_field = Some(key);
                    state.ref_select_claim();
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if let Some(index) = out.pick_remove {
            state.ref_select_remove(index);
        }
        if out.pick_done {
            state.finish_ref_select();
            self.picking_field = None;
            if let Err(why) = sync_operation_faces(state, id) {
                self.refuse(state, why);
            }
        }
        if out.changed {
            self.notice = None;
            let coalesce = format!("operation:{id}:form");
            if let Err(why) = update_operation(state, id, &params, Some(&coalesce)) {
                self.refuse(state, why);
            }
        }
        if let Some(index) = out.button_clicked.as_deref().and_then(|key| key.strip_prefix("removeChain")) {
            let patch = json!({ "targets": { "chains": { index: { "remove": true } } } });
            if let Err(why) = update_operation(state, id, &patch, None) {
                self.refuse(state, why);
            }
        }
        if out.exit_clicked {
            if state.ref_select_active() {
                state.finish_ref_select();
            }
            self.picking_field = None;
            self.open = None;
        }
        if was_picking && !state.ref_select_active() && self.picking_field.is_some() {
            self.picking_field = None;
            if let Err(why) = sync_operation_faces(state, id) {
                self.refuse(state, why);
            }
        }
        if !state.ref_select_active() {
            self.picking_field = None;
        }
    }

    /// Draw tool `id`'s dialog: the shared form over its params; Calculate
    /// feeds through C12's calculator; every edit one coalesced undo step.
    /// Draw the document machine `id`'s editor: the shared form over the
    /// model's machine; every edit one coalesced undo step, refused by field
    /// with nothing applied.
    fn show_machine_form(&mut self, ui: &mut egui::Ui, state: &mut EngineState, doc: &CamDocument, id: &str) {
        let Some(machine) = doc.machines.iter().find(|m| m.id.as_str() == id) else { return };
        let fields = cam_machine::machine_form_fields(machine);
        let mut params = cam_machine::machine_form_params(machine);
        let hidden = cam_machine::hidden_fields(&params);
        let title = format!("Machine {id}");
        let banner = self.notice.as_ref().map(|n| (n.as_str(), egui::Color32::from_rgb(0xd2, 0x99, 0x22)));
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(machine.name.as_str()),
            fields: &fields,
            hidden: Some(&hidden),
            banner,
            trailing: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "cam:",
            read_only: None,
            picking: None,
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("cam:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("cam:form:machine:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        self.push_overlay(state, doc, None);
        if out.changed {
            self.notice = None;
            let coalesce = format!("machine:{id}:form");
            if let Err(why) = update_machine(state, id, &params, Some(&coalesce)) {
                self.refuse(state, why);
            }
        }
        if out.exit_clicked {
            self.open = None;
        }
    }

    fn show_tool_form(&mut self, ui: &mut egui::Ui, state: &mut EngineState, store: &dyn ModelStore, doc: &CamDocument, id: &str) {
        let Some(tool) = doc.tool(id) else { return };
        let fields = cam_tool::tool_form_fields();
        let mut params = cam_tool::tool_form_params(tool);
        if let Some(material) = &self.feeds_material {
            params["feedsMaterial"] = json!(material);
        }
        let hidden = cam_tool::hidden_fields(&params);
        let title = format!("Tool {id}");
        let banner = self.notice.as_ref().map(|n| (n.as_str(), egui::Color32::from_rgb(0xd2, 0x99, 0x22)));
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(tool.name.as_str()),
            fields: &fields,
            hidden: Some(&hidden),
            banner,
            trailing: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "cam:",
            read_only: None,
            picking: None,
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("cam:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("cam:form:tool:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        self.push_overlay(state, doc, None);
        if out.changed {
            self.notice = None;
            // The work material is the calculator's input, not the tool's.
            self.feeds_material = params.get("feedsMaterial").and_then(Value::as_str).map(str::to_string);
            let coalesce = format!("tool:{id}:form");
            if let Err(why) = update_tool(state, id, &params, Some(&coalesce)) {
                self.refuse(state, why);
            }
        }
        if out.button_clicked.as_deref() == Some("calculateFeeds") {
            let material = params.get("feedsMaterial").and_then(Value::as_str).unwrap_or("aluminium").to_string();
            match calculate_tool_feeds(state, id, &material) {
                Ok(adjustments) => {
                    self.notice = Some(if adjustments.is_empty() { format!("feeds calculated for {material}") } else { format!("feeds calculated for {material}: {}", adjustments.join("; ")) });
                }
                Err(why) => self.refuse(state, why),
            }
        }
        if out.exit_clicked {
            self.open = None;
            self.user_library = None;
        }
        let _ = store;
    }

    /// Push (or clear) the viewport overlay for setup `id`: its stock box and
    /// its frame's triad. Unchanged geometry is not re-sent.
    fn push_overlay(&mut self, state: &mut EngineState, doc: &CamDocument, id: Option<String>) {
        self.refresh_geometry(state);
        let group = id
            .as_deref()
            .filter(|id| doc.setup(id).is_some())
            .and_then(|id| self.geometry.get(id))
            .map(|geometry| {
                let frame = geometry.frame.clone().unwrap_or_default();
                cam_setup::overlay_group(&frame, geometry.stock)
            })
            .unwrap_or_else(|| json!({ "name": "cam-setup" }));
        let key = (group.to_string(), state.cam.results_stamp, self.selected.clone(), self.scrub.map(f64::to_bits), self.hidden_stamp);
        if self.overlay_key.as_ref() == Some(&key) {
            return;
        }
        self.overlay_key = Some(key);
        // Every generated toolpath, the selected operation's at full strength
        // with the scrub marker; a result that is gone takes its group away.
        let mut groups = vec![group];
        let mut live: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        for (op, result) in &state.cam.results {
            if self.hidden_toolpaths.contains(op) {
                continue;
            }
            if let Some(toolpath) = &result.toolpath {
                let selected = self.selected.as_deref() == Some(op.as_str());
                let operation = doc.operation(op);
                let to_model = operation.and_then(|o| doc.setup(o.setup.as_str())).map(|s| s.machine_to_model()).unwrap_or_default();
                let profile = if selected && self.scrub.is_some() {
                    operation.and_then(|o| doc.tool(o.tool.as_str())).and_then(cam_generate::tool_profile)
                } else {
                    None
                };
                groups.push(cam_generate::toolpath_group(op, Some(toolpath), selected, if selected { self.scrub } else { None }, &to_model, profile.as_ref()));
                live.insert(op.clone());
            }
        }
        for gone in self.overlay_ops.difference(&live) {
            groups.push(cam_generate::toolpath_group(gone, None, false, None, &RigidTransform::default(), None));
        }
        self.overlay_ops = live;
        let json = json!({ "groups": groups }).to_string();
        if json != self.overlay {
            let _ = state.widgets.set_overlay_json(&json);
            state.dirty = true;
            self.overlay = json;
        }
    }

    /// The per-frame widget rects for the headed verifier.
    pub fn hits_json(&self) -> String {
        let mut all = self.hits.clone();
        all.extend(self.sim.hit_rects().iter().map(|(k, r)| (k.clone(), *r)));
        all.extend(self.post.hit_rects().iter().map(|(k, r)| (k.clone(), *r)));
        crate::automation::hit_rects::hits_json(&all)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame, whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
        self.sim.clear_hits();
    }
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "cam", prefix: "cam:auto:", meaning: "automatic programming preview, feature checkboxes, create and close", command: Some("cam_autoprogram") },
    HitKeyDoc { panel: "cam", prefix: "panel:clip", meaning: "the visible region of the pane (scroll_into_view scrolls a cam:* widget inside it)", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:new:setup", meaning: "add a setup (the toolbar's New setup)", command: Some("cam_add_setup") },
    HitKeyDoc { panel: "cam", prefix: "cam:new:tool", meaning: "add a tool to the document's library (the toolbar's New tool)", command: Some("cam_add_tool") },
    HitKeyDoc { panel: "cam", prefix: "cam:new:operation", meaning: "add an operation to the selected setup with the selected tool (the toolbar's New operation)", command: Some("cam_add_operation") },
    HitKeyDoc { panel: "cam", prefix: "cam:generate", meaning: "generate the toolpaths of every operation that needs it (the toolbar's Generate)", command: Some("cam_generate") },
    HitKeyDoc { panel: "cam", prefix: "cam:progress", meaning: "the generation line under the header: the operation generating, its phase and count, the queue; after a cancel, what was cancelled", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:cancel", meaning: "cancel the generation: the queue is dropped at once, the operation in flight stops at its next chunk where the runner can", command: Some("cam_cancel") },
    HitKeyDoc { panel: "cam", prefix: "cam:error", meaning: "why the document's `cam` block cannot be read (every edit is refused until it is repaired)", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:notice", meaning: "what the last action said that was not a record (a refusal, a count)", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:section:", meaning: "a section header (cam:section:setups, cam:section:programs, cam:section:tools, cam:section:machines) \u{2014} click folds it", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:machines:", meaning: "the Machines section's New machine menu (cam:machines:new) and, while it is open, one entry per built-in machine to copy (cam:machines:new:<built-in id>)", command: Some("cam_add_machine") },
    HitKeyDoc { panel: "cam", prefix: "cam:row:", meaning: "a setup, operation, NC program, tool or document machine row (cam:row:<id>) \u{2014} a click selects it (again to clear); right-click for its menu", command: Some("cam_select") },
    HitKeyDoc { panel: "cam", prefix: "cam:menuitem:", meaning: "one entry of the OPEN row menu (cam:menuitem:<id>:delete; a setup's or an operation's :edit; an operation's :toolpath = Hide / Show toolpath; a machine's :edit; a tool's :edit, :duplicate and :library) \u{2014} published only while the menu is up", command: Some("cam_remove") },
    HitKeyDoc { panel: "cam", prefix: "cam:form:", meaning: "the open setup, tool, operation or machine dialog (cam:form:setup:<id>, cam:form:tool:<id>, cam:form:operation:<id>, cam:form:machine:<id>, cam:form:feature, cam:form:return)", command: Some("cam_open") },
    HitKeyDoc { panel: "cam", prefix: "cam:field:", meaning: "one control of the open dialog (cam:field:<path>, nested paths joined by `.`, plus #activate on a reference field): a setup's name, wcsNumber, zRef, xRef, originRef, stockKind and its parameters, clearance, retract, feed, targets, protected; a tool's name, number, kind, description, material, the geometry, holderFamily/holderBore/holderHeight, spindleRpm, spindleDirection, cuttingFeed, plungeFeed, feedsMaterial, calculateFeeds, coolant; an operation's name, enabled, strategyKind, tool, feedsOverride, feeds.*, targets.targetSolids / driveFaces / avoidFaces / protectedSolids, targets.chainEdges / chainProfile (pickers that add a chain), the chain list targets.chains.<i>.source.kind / summary / closed / reversed / side / offset / regions / startSet / start and its targets.removeChain<i> button, heightsOverride, heights.*, tolerance, stockToLeave.*, strategy.* (its strategy's parameters) and linking.*; a document machine's name, kind, post, toolChangeSeconds, spindle.*, feed.*, accelerationSet, travelSet, travel.min / max, kinematics, yAxis, axes.<i>.name / direction / origin / limitsSet / limitsMin / limitsMax and collision (read only)", command: Some("cam_update_setup") },
    HitKeyDoc { panel: "cam", prefix: "cam:tools:", meaning: "the Tool Library section's actions: cam:tools:library (the Add-from-user-library menu), cam:tools:import (the file chooser), cam:tools:export (the document's tools as a library file), cam:tools:export-user", command: Some("cam_library") },
    HitKeyDoc { panel: "cam", prefix: "cam:library:", meaning: "one entry of the OPEN Add-from-user-library menu (cam:library:<library tool id>) \u{2014} published only while the menu is up", command: Some("cam_library") },
];

