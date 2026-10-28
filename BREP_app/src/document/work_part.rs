//! Work part: edit one part occurrence's own history in the context of its
//! assembly, at any depth of the product structure (NX "Make Work Part").
//!
//! IN PLACE (the default, a BOM double-click): the part's document is HOSTED by
//! the assembly tab. It is not a tab of its own; the tab strip, the tab's name,
//! Save and the dirty dot all stay the assembly's. [`Documents::engine`] answers
//! with the hosted part engine, so every modeling tool and the history panel
//! edit the part, while the rest of the assembly is drawn around it as a
//! translucent, non-selectable context scene in the same camera.
//!
//! NEW TAB (the row menu's "Open in new tab"): the same session, but the part
//! document is an ordinary tab and the assembly a background one.
//!
//! LIVE UPDATE. Every settled part change is pushed into the assembly's
//! embedded definitions (every copy with the same source identity, at every
//! depth) through `edit_document_json`, which re-runs the assembly on its own
//! runner. Mates are solved at the tail of that run, so components constrained
//! to the edited geometry move in the same update. When the assembly's run
//! lands the context scene is rebuilt from it (and re-mapped if the work
//! occurrence itself moved). A push is never queued behind a running assembly
//! run: while one is in flight the latest part state waits, so a burst of edits
//! costs one rerun of the newest state.
//!
//! UNDO. One part edit is one part undo step and one assembly undo step. The
//! session remembers the part states it pushed; a part undo (redo) that returns
//! to the previous (next) pushed state undoes (redoes) the assembly step
//! instead of pushing a new one, and a different edit after an undo discards
//! the redo branch of both. Coalesced part edits (one slider drag) replace the
//! assembly step they started rather than stacking. Returning adds nothing:
//! the assembly already holds one step per part edit.
use super::{Document, Documents};
use brep_render::{brep_kernel, camera::Aabb, engine_state::EngineState, scene::RenderScene};
use serde_json::Value;

type Matrix = [f64; 16];
const IDENTITY: Matrix = [
    1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
];

/// Where the work part's document lives while it is being edited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkPartMode {
    /// Hosted by the assembly tab (no tab switch).
    InPlace,
    /// An ordinary tab of its own; the assembly keeps running in the background.
    NewTab,
}

/// The entry at the end of a part-name chain in an assembly document.
pub(super) fn entry_at<'a>(document: &'a Value, names: &[String]) -> Option<&'a Value> {
    let (last, parents) = names.split_last()?;
    let mut owner = document;
    for name in parents {
        owner = &owner["partsLibrary"][name]["document"];
    }
    let entry = &owner["partsLibrary"][last];
    entry.is_object().then_some(entry)
}

/// One part state the session pushed into the assembly.
struct Pushed {
    signature: String,
    /// The part's undo-stack top when it was pushed: unchanged across two
    /// pushes means the second edit coalesced into the first's undo step.
    undo_top: Option<u64>,
    /// The parts this push re-signed and the mates it captured before its
    /// commit, from the document its assembly step starts from. A coalesced
    /// edit replaces the step from that same document, so it reuses them.
    capture: Option<(Vec<String>, brep_render::engine_state::replace_component::ReplaceCapture)>,
}

pub(super) struct WorkPart {
    pub assembly_id: u64,
    pub part_id: u64,
    /// `Some` while in place: the hosted part document.
    pub(super) hosted: Option<Box<Document>>,
    name: String,
    component_path: String,
    part_path: Vec<String>,
    source: Option<String>,
    world: Matrix,
    /// Pushed part states, oldest first; `[0]` is the state the session began
    /// with (nothing to undo past it).
    pushed: Vec<Pushed>,
    /// Pushed states undone in step with the part, newest last.
    redo: Vec<Pushed>,
    /// The part's (applied generation, history revision) last examined.
    seen: Option<(u64, u64)>,
    /// The assembly's (applied generation, history revision) the context was
    /// built from.
    context_seen: Option<(u64, u64)>,
    /// Constraints of the assembly that are not satisfied after the last
    /// update: `(id, type, status, message)`.
    problems: Vec<(String, String, String, String)>,
    /// How many pushes reached the assembly (automation readback).
    pushes: u64,
    /// The last push refusal, shown in the bar.
    error: Option<String>,
    /// A PLM part not checked out to this user: the offer to check it out
    /// now (asked once per session), and the checkout in flight.
    checkout: CheckoutOffer,
}

/// [`WorkPart::checkout`].
#[derive(Default)]
enum CheckoutOffer {
    /// Not asked yet (or not a PLM part: asked of nothing).
    #[default]
    Unasked,
    /// The question is on screen.
    Asking { part: String, revision: String },
    Running(crate::plm::PlmFuture<()>),
    /// Answered, landed, or nothing to ask; `Some` is a line for the bar.
    Done(Option<String>),
}

/// Resolve `component_path` (`ACOMP1:ACOMP2`) in the assembly: the
/// occurrence's world matrix, the part-name chain, the part's source key and
/// its embedded document.
pub(super) fn resolve_occurrence(
    root: &EngineState,
    root_document: &Value,
    component_path: &str,
) -> Result<(Matrix, Vec<String>, Option<String>, Value), String> {
    let mut document = root_document;
    let mut world = IDENTITY;
    let mut part_path = Vec::new();
    let mut source = None;
    let ids: Vec<&str> = component_path.split(':').collect();
    let mut top: Option<&brep_render::pipeline::ComponentSnapshot> = None;
    for (depth, &id) in ids.iter().enumerate() {
        let feature = document["features"]
            .as_array()
            .and_then(|features| {
                features.iter().find(|f| {
                    matches!(f["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT"))
                        && f["inputParams"]["id"].as_str() == Some(id)
                })
            })
            .ok_or_else(|| format!("component '{id}' is missing"))?;
        let name = feature["inputParams"]["partName"]
            .as_str()
            .ok_or("component has no part name")?;
        // Every depth reads the SOLVED pose: the assembly's mates move a
        // top-level component, and a sub-assembly's own mates (solved inside
        // it, rigid to the parent) can move a nested occurrence away from the
        // transform its document authors. The top-level record carries where
        // each nested occurrence ended up, in its own frame; the authored
        // transform is only the fallback for a record that has none.
        world = if depth == 0 {
            let record = root
                .assembly_components()
                .iter()
                .find(|c| c.id == id)
                .ok_or("component is not present in the current assembly")?;
            top = Some(record);
            record.transform
        } else {
            match top.and_then(|record| Some((record, record.nested_poses.get(&ids[1..=depth].join(":"))?))) {
                Some((record, solved)) => multiply(record.transform, *solved),
                None => multiply(world, component_pose(document, &feature["inputParams"]["transform"])?),
            }
        };
        part_path.push(name.to_string());
        let entry = &document["partsLibrary"][name];
        source = entry["sourceKey"]
            .as_str()
            .filter(|s| !s.is_empty() && !s.starts_with('@'))
            .map(str::to_string);
        document = &entry["document"];
        if !document.is_object() {
            return Err(format!("part '{name}' has no embedded document"));
        }
    }
    Ok((world, part_path, source, document.clone()))
}

/// The assembly drawn around the work part: every visible solid OUTSIDE the
/// work occurrence, mapped into the part's frame, without kernel handles.
///
/// A ghost built exactly as it stood in `previous` keeps that one's revision,
/// so the GPU keeps its buffers. A run of the assembly gives every solid it
/// rebuilt a new revision, and on l500 each edit rebuilds all five modules:
/// the 490 context solids were uploaded again on the frame the run landed
/// (~60 ms), the same meshes nearly all of them.
fn context_scene(host: &EngineState, component_path: &str, world: Matrix, previous: &RenderScene) -> RenderScene {
    let inverse = inverse_rigid(world);
    let prefix = format!("{component_path}:");
    // A remap moves every vertex without a new host revision, so the GPU
    // cache must see a different one: derive it from the host revision and
    // the frame (high bit set, clear of the scene's own monotonic counter).
    let frame = world
        .iter()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, v| (h ^ v.to_bits()).wrapping_mul(0x100_0000_01b3));
    let mut context = RenderScene::new();
    for solid in host
        .scene
        .solids()
        .iter()
        .filter(|solid| solid.visible && !solid.name.starts_with(&prefix))
    {
        let mut ghost = solid.clone();
        ghost.source_handle = 0;
        ghost.revision = ((solid.revision ^ frame).wrapping_mul(0x9e37_79b9_7f4a_7c15)) | (1 << 63);
        ghost.shape_revision = 0;
        ghost.pose = None;
        ghost.bbox = Aabb::empty();
        for point in &mut ghost.mesh.positions {
            let mapped = point_at(inverse, point.map(f64::from));
            ghost.bbox.expand(mapped);
            *point = mapped.map(|v| v as f32);
        }
        for normal in &mut ghost.mesh.normals {
            *normal = vector_at(inverse, normal.map(f64::from)).map(|v| v as f32);
        }
        // Context edges stay, in the part's frame, so a sketch can copy one;
        // vertices and exact kernel handles do not.
        for edge in &mut ghost.edges {
            for point in &mut edge.polyline {
                *point = point_at(inverse, point.map(f64::from)).map(|v| v as f32);
            }
        }
        ghost.vertices.clear();
        if let Some(before) = previous.solid(&ghost.name) {
            let fresh = ghost.revision;
            ghost.revision = before.revision;
            if ghost != *before {
                ghost.revision = fresh;
            }
        }
        context.insert_solid(ghost);
    }
    context
}

fn edit_key(engine: &EngineState) -> (u64, u64) {
    (engine.applied_generation(), engine.history_revision())
}

impl Documents {
    /// The part an occurrence places, at any depth (`ACOMP1:ACOMP2`), in the
    /// active tab: its part-library name and its source file (`None` or empty:
    /// a part with no file of its own).
    pub fn occurrence_part(&self, component_path: &str) -> Result<(String, Option<String>), String> {
        let root = self.engine();
        let document: Value = serde_json::from_str(&root.history_request_json()).map_err(|e| e.to_string())?;
        let (_, names, source, _) = resolve_occurrence(root, &document, component_path)?;
        Ok((names.last().cloned().unwrap_or_default(), source))
    }

    /// Make `component_path` the Work part, in place (no tab switch).
    pub fn begin_work_part(&mut self, component_path: &str) -> Result<(), String> {
        self.begin_work_part_as(component_path, WorkPartMode::InPlace)
    }

    pub fn begin_work_part_as(
        &mut self,
        component_path: &str,
        mode: WorkPartMode,
    ) -> Result<(), String> {
        if self.work_part.is_some() {
            // Another line of the same assembly: switch the Work part.
            let host = self.work_part.as_ref().map(|s| s.assembly_id);
            self.finish_work_part()?;
            if let Some(index) = host.and_then(|id| self.open.iter().position(|d| d.id() == id)) {
                self.activate(index);
            }
        }
        let root = &self.open[self.active].engine;
        if root.run_pending()
            || root.sketch_mode()
            || root.sketch3d_mode()
            || root.ref_select_active()
        {
            return Err("finish the current operation before choosing a Work part".into());
        }
        if !self.active().access().is_editable() {
            return Err("the assembly is read-only".into());
        }
        let assembly_id = self.active_id();
        let root_document: Value =
            serde_json::from_str(&root.history_request_json()).map_err(|e| e.to_string())?;
        let (world, part_path, source, document) =
            resolve_occurrence(root, &root_document, component_path)?;
        let context = context_scene(root, component_path, world, &RenderScene::new());
        let inverse = inverse_rigid(world);
        let mut camera = root.camera.clone();
        camera.eye = point_at(inverse, camera.eye);
        camera.target = point_at(inverse, camera.target);
        camera.up = vector_at(inverse, camera.up);
        let existing = source.as_deref().and_then(|name| self.index_of(name));
        if existing == Some(self.active_index()) {
            return Err("the component source refers to its assembly".into());
        }
        // A part also open in its own tab is fine in place: the two are kept
        // one by the live link (`super::live_link`) on every tab switch.
        let load = |docs: &Documents| -> Result<Document, String> {
            let mut engine = docs.spawn_engine();
            engine.set_history_json(&document.to_string())?;
            // A part without its own workbench must not inherit Assembly tools.
            if document["workbench"].as_str().is_none() {
                let is_assembly = document["features"].as_array().is_some_and(|features| {
                    features
                        .iter()
                        .any(|f| matches!(f["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")))
                });
                engine.settings.workbench =
                    if is_assembly { "assembly" } else { "modeling" }.into();
            }
            let mut doc = Document::new(engine);
            doc.set_name(source.clone());
            doc.mark_clean();
            Ok(doc)
        };
        let (hosted, part_id) = match (mode, existing) {
            (WorkPartMode::NewTab, Some(index)) => {
                if !self.open[index].access().is_editable() {
                    return Err("the part is read-only".into());
                }
                let id = self.open[index].id();
                self.activate(index);
                (None, id)
            }
            (WorkPartMode::NewTab, None) => {
                let doc = load(self).inspect_err(|_| self.install_active_library())?;
                let id = doc.id();
                self.open_document(doc);
                (None, id)
            }
            (WorkPartMode::InPlace, _) => {
                let doc = load(self).inspect_err(|_| self.install_active_library())?;
                let id = doc.id();
                (Some(Box::new(doc)), id)
            }
        };
        let mut session = WorkPart {
            assembly_id,
            part_id,
            hosted,
            name: part_path.last().cloned().unwrap_or_default(),
            component_path: component_path.into(),
            part_path,
            source,
            world,
            pushed: Vec::new(),
            redo: Vec::new(),
            seen: None,
            context_seen: None,
            problems: Vec::new(),
            pushes: 0,
            error: None,
            checkout: CheckoutOffer::default(),
        };
        let host_key = edit_key(&self.open[self.active_host_index(&session)].engine);
        session.context_seen = Some(host_key);
        self.work_part = Some(session);
        let part = self.work_engine_mut().expect("work part engine");
        part.context_scene = context;
        part.sync_context_highlight();
        part.camera = camera;
        part.clear_selection();
        part.dirty = true;
        let baseline = Pushed {
            signature: super::signature(part),
            undo_top: part.history.undo_top(),
            capture: None,
        };
        let seen = edit_key(part);
        let session = self.work_part.as_mut().unwrap();
        session.pushed.push(baseline);
        session.seen = Some(seen);
        self.install_active_library();
        Ok(())
    }

    fn active_host_index(&self, session: &WorkPart) -> usize {
        self.open
            .iter()
            .position(|d| d.id() == session.assembly_id)
            .unwrap_or(self.active)
    }

    /// The hosted part document, while a Work part is edited in place.
    pub(super) fn hosted(&self) -> Option<&Document> {
        self.work_part.as_ref().and_then(|s| s.hosted.as_deref())
    }

    pub(super) fn hosted_mut(&mut self) -> Option<&mut Document> {
        self.work_part.as_mut().and_then(|s| s.hosted.as_deref_mut())
    }

    fn work_engine_mut(&mut self) -> Option<&mut EngineState> {
        let session = self.work_part.as_mut()?;
        if let Some(doc) = session.hosted.as_deref_mut() {
            return Some(&mut doc.engine);
        }
        let id = session.part_id;
        self.open.iter_mut().find(|d| d.id() == id).map(|d| &mut d.engine)
    }

    fn work_engine(&self) -> Option<&EngineState> {
        let session = self.work_part.as_ref()?;
        if let Some(doc) = session.hosted.as_deref() {
            return Some(&doc.engine);
        }
        self.open.iter().find(|d| d.id() == session.part_id).map(|d| &d.engine)
    }

    /// Whether a Work part is being edited in place in the active tab.
    pub fn work_part_in_place(&self) -> bool {
        self.hosted().is_some()
    }

    /// The assembly engine of the active Work part session (the tab's own
    /// engine while in place).
    pub fn work_part_assembly(&self) -> Option<&EngineState> {
        let session = self.work_part.as_ref()?;
        self.open.iter().find(|d| d.id() == session.assembly_id).map(|d| &d.engine)
    }

    pub fn work_part_assembly_mut(&mut self) -> Option<&mut EngineState> {
        let id = self.work_part.as_ref()?.assembly_id;
        self.open.iter_mut().find(|d| d.id() == id).map(|d| &mut d.engine)
    }

    /// The component path being edited, if any.
    pub fn work_part_path(&self) -> Option<&str> {
        self.work_part.as_ref().map(|s| s.component_path.as_str())
    }

    /// Push the part's settled state into the assembly and keep the context
    /// current. Called every frame after the pumps ([`Documents::pump_all`]).
    pub(super) fn sync_work_part(&mut self) {
        let Some(session) = self.work_part.as_ref() else {
            return;
        };
        let Some(host) = self.open.iter().position(|d| d.id() == session.assembly_id) else {
            return;
        };
        let Some(part) = self.work_engine() else {
            return;
        };
        let busy = part.run_pending()
            || part.sketch_mode()
            || part.sketch3d_mode()
            || part.ref_select_active();
        let key = edit_key(part);
        if !busy && Some(key) != session.seen && !self.open[host].engine.run_pending() {
            // Coalesce: while the assembly is still running the previous push,
            // `seen` stays put so the NEWEST part state goes next frame.
            let _ = self.push_work_part(false);
        }
        self.refresh_context();
    }

    /// Push the work part's current state, if it differs from the last pushed
    /// one. `force` pushes even while the assembly is running (Return).
    fn push_work_part(&mut self, force: bool) -> Result<(), String> {
        let session = self.work_part.as_ref().ok_or("no Work part is active")?;
        let host = self
            .open
            .iter()
            .position(|d| d.id() == session.assembly_id)
            .ok_or("assembly tab is missing")?;
        if !force && self.open[host].engine.run_pending() {
            return Ok(());
        }
        let part = self.work_engine().ok_or("part document is missing")?;
        let key = edit_key(part);
        let signature = super::signature(part);
        let undo_top = part.history.undo_top();
        let edited: Value =
            serde_json::from_str(&part.history_request_json()).map_err(|e| e.to_string())?;
        let session = self.work_part.as_mut().unwrap();
        session.seen = Some(key);
        if session.pushed.last().is_some_and(|p| p.signature == signature) {
            return Ok(());
        }
        let pushed_len = session.pushed.len();
        // A part undo back to the previous pushed state: undo the assembly step.
        if pushed_len >= 2 && session.pushed[pushed_len - 2].signature == signature {
            let step = session.pushed.pop().unwrap();
            session.redo.push(step);
            self.open[host].engine.undo();
            self.install_active_library();
            self.work_part.as_mut().unwrap().context_seen = None;
            return Ok(());
        }
        // A part redo to the next pushed state: redo the assembly step.
        if session.redo.last().is_some_and(|p| p.signature == signature) {
            let step = session.redo.pop().unwrap();
            session.pushed.push(step);
            self.open[host].engine.redo();
            self.install_active_library();
            self.work_part.as_mut().unwrap().context_seen = None;
            return Ok(());
        }
        // The same part undo step as the last push (a coalesced drag): replace
        // that assembly step instead of stacking another.
        let replace = pushed_len >= 2
            && undo_top.is_some()
            && session.pushed.last().is_some_and(|p| p.undo_top == undo_top);
        if !self.open[host].access().is_editable() {
            let session = self.work_part.as_mut().unwrap();
            session.error = Some("the assembly is read-only; part edits are not applied".into());
            return Err("the assembly is read-only".into());
        }
        // Replacing a coalesced step: its capture is reused, so the undo
        // runs nothing (the commit's run is the one the assembly needs); the
        // replay a capture would take with that run in flight goes too.
        let reused = if replace { session.pushed.last().and_then(|p| p.capture.clone()) } else { None };
        if replace {
            if reused.is_some() {
                self.open[host].engine.undo_before_edit();
            } else {
                self.open[host].engine.undo();
            }
        }
        let path = session.part_path.clone();
        let source = session.source.clone();
        let mut assembly_document: Value =
            serde_json::from_str(&self.open[host].engine.history_request_json())
                .map_err(|e| e.to_string())?;
        let mut notes = Vec::new();
        let signatures_before = library_signatures(&assembly_document);
        let result = refresh_parts(&mut assembly_document, &path, source.as_deref(), &edited, &mut notes)
            .and_then(|changed| {
                if changed {
                    Ok(())
                } else {
                    Err("the assembly no longer holds this part's definition".to_string())
                }
            })
            .and_then(|()| {
                // Mates on the edited parts' occurrences are checked once the
                // run lands: a face the edit renamed on the same plane is
                // re-attached, anything ambiguous is held for Repair.
                let after = library_signatures(&assembly_document);
                let parts: Vec<String> =
                    after.iter().filter(|(name, sig)| signatures_before.get(*name) != Some(*sig)).map(|(name, _)| name.clone()).collect();
                // The capture reads the HOST's assembly session, replayed on
                // this thread: with the Work part active the kernel store holds
                // the part's library, and the last replay may be another
                // document's, so both are put back to the host's first.
                super::install_library_of(&self.open[host]);
                let engine = &mut self.open[host].engine;
                engine.forget_assembly_session();
                let capture = match reused {
                    Some((reused_parts, capture)) if reused_parts == parts => capture,
                    stale => {
                        // Other parts this time: capture from the undone
                        // document, run as the plain undo would have.
                        if stale.is_some() {
                            engine.rebuild_current_history();
                        }
                        engine.capture_part_edit(&engine.components_of_parts(&parts))
                    }
                };
                let kept = (parts.clone(), capture.clone());
                engine.commit_part_edit(capture, assembly_document.clone(), &parts).map(|()| kept)
            });
        self.install_active_library();
        let session = self.work_part.as_mut().unwrap();
        match result {
            Ok(capture) => {
                if replace {
                    session.pushed.pop();
                }
                session.pushed.push(Pushed { signature, undo_top, capture: Some(capture) });
                session.redo.clear();
                session.pushes += 1;
                session.error = None;
                session.context_seen = None;
                for note in notes {
                    self.open[host].engine.push_notice(format!("Work part: {note}"));
                }
                Ok(())
            }
            Err(error) => {
                if replace {
                    self.open[host].engine.redo();
                    self.install_active_library();
                }
                let session = self.work_part.as_mut().unwrap();
                session.error = Some(error.clone());
                Err(error)
            }
        }
    }

    /// Whether the Work part session still owes the assembly an update: a
    /// settled part state not pushed yet, an assembly run in flight, or a
    /// context not rebuilt from the assembly's latest run.
    pub fn work_part_settling(&self) -> bool {
        // Going up to a sub-assembly: it begins when the assembly's run lands.
        if self.work_part_next.is_some() {
            return true;
        }
        let Some(session) = self.work_part.as_ref() else {
            return false;
        };
        let (Some(part), Some(host)) = (self.work_engine(), self.work_part_assembly()) else {
            return false;
        };
        let editing = part.sketch_mode() || part.sketch3d_mode() || part.ref_select_active();
        host.run_pending()
            || (!editing && Some(edit_key(part)) != session.seen && session.error.is_none())
            || session.context_seen != Some(edit_key(host))
    }

    /// The source files the ACTIVE tab's next Save writes besides itself:
    /// every owned document its edits changed ([`super::owned_save`]).
    pub fn work_part_modified_sources(&self) -> Vec<String> {
        self.owned_modified_keys()
    }

    /// Push the Work part's latest state now, even while the assembly is still
    /// running (Save and the document readback call this). A refusal is kept
    /// for the bar; the part edits stay in the Work part.
    pub fn flush_work_part(&mut self) {
        if self.work_part.is_some() {
            let _ = self.push_work_part(true);
        }
    }

    /// Rebuild the context scene once the assembly's latest run has landed.
    fn refresh_context(&mut self) {
        let Some(session) = self.work_part.as_ref() else {
            return;
        };
        let Some(host) = self.open.iter().position(|d| d.id() == session.assembly_id) else {
            return;
        };
        let engine = &self.open[host].engine;
        if engine.run_pending() || session.context_seen == Some(edit_key(engine)) {
            return;
        }
        let key = edit_key(engine);
        let world = serde_json::from_str::<Value>(&engine.history_request_json())
            .ok()
            .and_then(|document| resolve_occurrence(engine, &document, &session.component_path).ok())
            .map(|(world, ..)| world)
            .unwrap_or(session.world);
        let previous = self.work_engine().map(|part| &part.context_scene);
        let context = context_scene(engine, &session.component_path, world, previous.unwrap_or(&RenderScene::new()));
        let mut statuses = self.open[host].engine.assembly_statuses_value();
        let mut problems: Vec<(String, String, String, String)> = statuses
            .as_array_mut()
            .map(|rows| {
                rows.iter()
                    .filter(|row| {
                        let status = row["status"].as_str().unwrap_or("");
                        row["enabled"].as_bool() != Some(false)
                            && !matches!(status, "satisfied" | "disabled" | "over" | "")
                    })
                    .map(|row| {
                        let text = |k: &str| row[k].as_str().unwrap_or("").to_string();
                        (text("id"), text("type"), text("status"), text("message"))
                    })
                    .collect()
            })
            .unwrap_or_default();
        // A mate the push held for Repair is disabled, so its status no longer
        // says so: it is a problem all the same until it is repaired.
        for held in self.open[host].engine.replace_unresolved().as_array().into_iter().flatten() {
            let id = held["id"].as_str().unwrap_or("").to_string();
            if !problems.iter().any(|(known, ..)| *known == id) {
                let message = "held for Repair: the face or edge it used is gone".to_string();
                problems.push((id, held["type"].as_str().unwrap_or("").to_string(), "held".into(), message));
            }
        }
        let moved = world != session.world;
        let session = self.work_part.as_mut().unwrap();
        session.context_seen = Some(key);
        session.problems = problems;
        if moved {
            // The work occurrence itself moved (a mate re-posed it): keep the
            // view where it was in the assembly by re-mapping the camera too.
            let delta = multiply(inverse_rigid(world), session.world);
            session.world = world;
            if let Some(part) = self.work_engine_mut() {
                part.camera.eye = point_at(delta, part.camera.eye);
                part.camera.target = point_at(delta, part.camera.target);
                part.camera.up = vector_at(delta, part.camera.up);
            }
        }
        if let Some(part) = self.work_engine_mut() {
            part.context_scene = context;
            part.sync_context_highlight();
            part.dirty = true;
        }
    }

    /// Called before leaving the work context, including ordinary tab changes.
    pub(super) fn finish_work_part(&mut self) -> Result<(), String> {
        let Some(session) = &self.work_part else {
            return Ok(());
        };
        let assembly = self
            .open
            .iter()
            .position(|d| d.id() == session.assembly_id)
            .ok_or("assembly tab is missing")?;
        if self.work_engine().is_none() {
            self.work_part = None;
            return Err("part document is missing".into());
        }
        {
            let part = self.work_engine_mut().unwrap();
            if part.ref_select_active() {
                part.cancel_ref_select();
            }
            // Sketch edits are already kept in the document. Finish the editor
            // so a standalone tab does not retain context references.
            if part.sketch_mode() {
                part.exit_sketch_mode(true);
            }
            if part.sketch3d_mode() {
                part.exit_sketch3d_mode(true);
            }
        }
        let part = self.work_engine().unwrap();
        let changed = self
            .work_part
            .as_ref()
            .and_then(|s| s.pushed.last())
            .is_none_or(|p| p.signature != super::signature(part));
        if changed {
            if !self.open[assembly].access().is_editable() {
                return Err("assembly is read-only; part edits remain open".into());
            }
            self.push_work_part(true).map_err(|error| {
                format!("{error}; edits remain in the Work part")
            })?;
        }
        let session = self.work_part.take().unwrap();
        let world = session.world;
        let camera = match &session.hosted {
            Some(doc) => doc.engine.camera.clone(),
            None => {
                let part = self.open.iter_mut().find(|d| d.id() == session.part_id).unwrap();
                part.engine.context_scene.clear();
                part.engine.sync_context_highlight();
                part.engine.dirty = true;
                part.engine.camera.clone()
            }
        };
        let host = &mut self.open[assembly].engine;
        host.camera.eye = point_at(world, camera.eye);
        host.camera.target = point_at(world, camera.target);
        host.camera.up = vector_at(world, camera.up);
        host.camera.projection = camera.projection;
        host.dirty = true;
        self.install_active_library();
        Ok(())
    }

    /// Go up the Work part's path to `depth`: 0 is the assembly itself, `k`
    /// the occurrence of its first `k` components (a sub-assembly above the
    /// part). The part's edits are pushed first, and the sub-assembly becomes
    /// the Work part once that update has landed, so it is placed where the
    /// solved assembly puts it (`Self::begin_next_work_part`).
    pub fn work_part_up_to(&mut self, depth: usize) -> Result<(), String> {
        let session = self.work_part.as_ref().ok_or("no Work part is active")?;
        let ids: Vec<&str> = session.component_path.split(':').collect();
        if depth >= ids.len() {
            return Ok(());
        }
        let next = (
            session.assembly_id,
            ids[..depth].join(":"),
            if session.hosted.is_some() { WorkPartMode::InPlace } else { WorkPartMode::NewTab },
        );
        self.return_to_assembly()?;
        if depth > 0 {
            self.work_part_next = Some(next);
            self.begin_next_work_part();
        }
        Ok(())
    }

    /// One level up: the sub-assembly holding the Work part, or the assembly.
    pub fn work_part_up(&mut self) -> Result<(), String> {
        let depth = self.work_part.as_ref().ok_or("no Work part is active")?.part_path.len();
        self.work_part_up_to(depth.saturating_sub(1))
    }

    /// Begin the Work part [`Self::work_part_up_to`] left waiting, once its
    /// assembly is the active tab with no run in flight. Dropped if the user
    /// moved to another tab or chose another Work part meanwhile.
    pub(super) fn begin_next_work_part(&mut self) {
        let Some((assembly, _, _)) = &self.work_part_next else {
            return;
        };
        if self.work_part.is_some() || self.active_id() != *assembly {
            self.work_part_next = None;
            return;
        }
        if self.engine().run_pending() {
            return;
        }
        let (_, path, mode) = self.work_part_next.take().unwrap();
        if let Err(error) = self.begin_work_part_as(&path, mode) {
            self.engine_mut().push_notice(format!("Work part: {error}"));
        }
    }

    pub fn return_to_assembly(&mut self) -> Result<(), String> {
        let id = self
            .work_part
            .as_ref()
            .ok_or("no Work part is active")?
            .assembly_id;
        self.finish_work_part()?;
        let index = self
            .open
            .iter()
            .position(|d| d.id() == id)
            .ok_or("assembly tab is missing")?;
        self.activate(index);
        Ok(())
    }

    /// The Work part session as automation state (`null` when inactive).
    pub fn work_part_state(&self) -> Value {
        let Some(s) = self.work_part.as_ref() else {
            return Value::Null;
        };
        let assembly = self.work_part_assembly();
        let part = self.work_engine();
        // Context solids in ASSEMBLY coordinates, so a script can read where
        // the other components are after an update.
        // Rebuilt only when the host scene changed (a solid's revision or
        // visibility, or the set): built every frame it cost 35-58 ms at 500
        // parts, about half of a Work part edit's measured time.
        let context: Value = assembly
            .map(|engine| {
                let fingerprint = engine.scene.solids().iter().fold(engine.scene.solids().len() as u64, |h, solid| {
                    (h ^ solid.revision ^ u64::from(solid.visible)).wrapping_mul(0x100_0000_01b3)
                });
                let key = (s.component_path.clone(), fingerprint);
                if let Some((cached, value)) = self.work_part_context.borrow().as_ref() {
                    if *cached == key {
                        return value.clone();
                    }
                }
                let prefix = format!("{}:", s.component_path);
                let map: serde_json::Map<String, Value> = engine
                    .scene
                    .solids()
                    .iter()
                    .filter(|solid| solid.visible && !solid.name.starts_with(&prefix))
                    .map(|solid| {
                        (solid.name.clone(), serde_json::json!({"min": solid.bbox.min, "max": solid.bbox.max}))
                    })
                    .collect();
                let value = Value::Object(map);
                *self.work_part_context.borrow_mut() = Some((key, value.clone()));
                value
            })
            .unwrap_or_else(|| Value::Object(Default::default()));
        let [r, g, b, _] = brep_render::render::WORK_PART_CONTEXT_RGBA;
        let (shown, alpha) = part.map_or((true, 0.0), |engine| engine.context_display());
        let style = serde_json::json!({"rgba": [r, g, b, alpha], "alpha": alpha, "shown": shown, "depthWrite": false, "dimmed": alpha < 1.0});
        serde_json::json!({
            "name": s.name,
            "componentPath": s.component_path,
            "partPath": s.part_path,
            "mode": if s.hosted.is_some() { "inPlace" } else { "newTab" },
            "contextSolids": part.map_or(0, |engine| engine.context_scene.solids().len()),
            "contextSelectable": true,
            "contextSelection": part.map_or(serde_json::Value::Null, |engine| engine.context_selection_json()),
            "contextHover": part.map_or(serde_json::Value::Null, |engine| engine.context_hover_json()),
            "contextStyle": style,
            "context": context,
            "modifiedSources": self.owned_modified_keys_of(s.assembly_id),
            "assemblyId": s.assembly_id,
            "partId": s.part_id,
            "pushes": s.pushes,
            "pending": assembly.is_some_and(EngineState::run_pending),
            "undoSteps": s.pushed.len().saturating_sub(1),
            "redoSteps": s.redo.len(),
            "error": s.error,
            "constraintProblems": s.problems.iter().map(|(id, ty, status, message)| {
                serde_json::json!({"id": id, "type": ty, "status": status, "message": message})
            }).collect::<Vec<_>>(),
        })
    }

    /// Starting to edit a PLM part in context that is not checked out to this
    /// user: offer the checkout right there, Yes by default (owner,
    /// 2026-10-20). Locked by someone else or not editable: said in the bar;
    /// the edits stay in the assembly. A file store never reaches this.
    fn offer_checkout(&mut self, ui: &mut eframe::egui::Ui, store: &dyn crate::store::ModelStore, hits: &mut Value) {
        use eframe::egui;
        let Some(session) = self.work_part.as_mut() else {
            return;
        };
        if matches!(session.checkout, CheckoutOffer::Unasked) {
            session.checkout = CheckoutOffer::Done(None);
            let key = session.source.clone().unwrap_or_default();
            if let (Some(_), Some((part, revision))) = (store.plm_client(), crate::plm::bom::revision_of_document(&key)) {
                session.checkout = match store.plm_revision(&key) {
                    Some(entry) if entry.editable && entry.locked_by_me => CheckoutOffer::Done(None),
                    Some(entry) if !entry.editable => CheckoutOffer::Done(Some(format!(
                        "{key}: revision {} is {} — your edits stay in the assembly", entry.revision_label, entry.lifecycle
                    ))),
                    Some(entry) if entry.locked_by.is_some() => CheckoutOffer::Done(Some(format!(
                        "{key} is checked out by {} — your edits stay in the assembly", entry.locked_by.unwrap_or_default()
                    ))),
                    _ => CheckoutOffer::Asking { part, revision },
                };
            }
        }
        let name = session.name.clone();
        match &mut session.checkout {
            CheckoutOffer::Asking { part, revision } => {
                let (part, revision) = (part.clone(), revision.clone());
                let mut answer = None;
                let modal = egui::Modal::new(egui::Id::new("brep-work-part-checkout")).show(ui.ctx(), |ui| {
                    ui.heading("Check out before editing?");
                    ui.label(format!("{name} is not checked out to you. Check it out now, so Save can write it?"));
                    ui.horizontal(|ui| {
                        let yes = ui.button("Check out");
                        hits["checkout:yes"] = serde_json::json!([yes.rect.min.x, yes.rect.min.y, yes.rect.width(), yes.rect.height()]);
                        if yes.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            answer = Some(true);
                        }
                        let no = ui.button("Not now");
                        hits["checkout:no"] = serde_json::json!([no.rect.min.x, no.rect.min.y, no.rect.width(), no.rect.height()]);
                        if no.clicked() {
                            answer = Some(false);
                        }
                    });
                });
                let answer = answer.or_else(|| modal.should_close().then_some(false));
                match (answer, store.plm_client()) {
                    (Some(true), Some(client)) => {
                        use crate::panels::plm::PlmLifecycle;
                        session.checkout = CheckoutOffer::Running(client.checkout(&part, &revision));
                    }
                    (Some(_), _) => session.checkout = CheckoutOffer::Done(Some(format!("{name} is not checked out; Save will offer it"))),
                    (None, _) => {}
                }
            }
            CheckoutOffer::Running(future) => {
                let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
                match std::future::Future::poll(future.as_mut(), &mut cx) {
                    std::task::Poll::Ready(Ok(())) => {
                        session.checkout = CheckoutOffer::Done(Some(format!("{name} is checked out to you")));
                        store.refresh_plm_index(&[session.source.clone().unwrap_or_default()], false);
                    }
                    std::task::Poll::Ready(Err(error)) => {
                        session.checkout = CheckoutOffer::Done(Some(format!("{name} could not be checked out: {error}")));
                    }
                    std::task::Poll::Pending => ui.ctx().request_repaint_after(std::time::Duration::from_millis(50)),
                }
            }
            CheckoutOffer::Unasked | CheckoutOffer::Done(_) => {}
        }
    }

    pub fn show_work_part_bar(&mut self, ui: &mut eframe::egui::Ui, store: &dyn crate::store::ModelStore) {
        // The readback walks every assembly solid: only for automation.
        let state = crate::automation::registry::enabled().then(|| self.work_part_state());
        let mut hits = serde_json::json!({});
        self.offer_checkout(ui, store, &mut hits);
        // The checkout question is a modal: the bar behind it takes no click,
        // so its widgets are not published while it is up (a script's click on
        // `return` would otherwise report a hit that did nothing).
        let modal = self.work_part.as_ref().is_some_and(|s| matches!(s.checkout, CheckoutOffer::Asking { .. }));
        if let Some(session) = &self.work_part {
            // The breadcrumb: the assembly, each sub-assembly above the part,
            // the part. Every level above the part can be clicked.
            let assembly = self.open.iter().find(|d| d.id() == session.assembly_id).map_or_else(|| "assembly".into(), |d| d.title());
            let crumbs: Vec<String> = std::iter::once(assembly).chain(session.part_path.iter().cloned()).collect();
            let component_path = session.component_path.clone();
            let problems: Vec<String> = session
                .problems
                .iter()
                .map(|(id, ty, status, message)| {
                    if message.is_empty() {
                        format!("{id} ({ty}): {status}")
                    } else {
                        format!("{id} ({ty}): {status}, {message}")
                    }
                })
                .collect();
            let error = session.error.clone();
            let checkout = match &session.checkout {
                CheckoutOffer::Done(line) => line.clone(),
                CheckoutOffer::Running(_) => Some("checking out…".into()),
                _ => None,
            };
            let modified: Vec<String> = self.owned_modified_keys_of(session.assembly_id);
            let context_part = self.work_engine().and_then(|engine| engine.context_selection().map(|c| c.solid.rsplit(':').next().unwrap_or(&c.solid).to_string()));
            let display = self.work_engine().map(|engine| engine.context_display());
            let mut new_display = display;
            // Some(true): sketch on the face; Some(false): datum only.
            let mut context_action: Option<bool> = None;
            let mut finish = false;
            // A breadcrumb level clicked (or Up): its depth.
            let mut up_to: Option<usize> = None;
            eframe::egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("Work part:");
                    let last = crumbs.len() - 1;
                    for (depth, crumb) in crumbs.iter().enumerate() {
                        if depth > 0 {
                            ui.weak("›");
                        }
                        if depth == last {
                            ui.strong(crumb);
                            continue;
                        }
                        let link = ui.link(crumb).on_hover_text(if depth == 0 {
                            "Return to the assembly"
                        } else {
                            "Make this sub-assembly the Work part"
                        });
                        hits[format!("crumb:{depth}")] = serde_json::json!([link.rect.min.x, link.rect.min.y, link.rect.width(), link.rect.height()]);
                        if link.clicked() {
                            up_to = Some(depth);
                        }
                    }
                    ui.weak(format!("({component_path})"));
                    // Nested: one level up is the sub-assembly holding the part.
                    if last > 1 {
                        let button = ui.button("Up").on_hover_text(format!("Make {} the Work part", crumbs[last - 1]));
                        hits["up"] = serde_json::json!([button.rect.min.x, button.rect.min.y, button.rect.width(), button.rect.height()]);
                        if button.clicked() {
                            up_to = Some(last - 1);
                        }
                    }
                    let button = ui
                        .button("Return to assembly")
                        .on_hover_text("Make the assembly the work context again (Esc)");
                    hits["return"] = serde_json::json!([
                        button.rect.min.x,
                        button.rect.min.y,
                        button.rect.width(),
                        button.rect.height()
                    ]);
                    finish = button.clicked();
                    // The rest of the assembly: shown for reference (it updates
                    // as you edit), or hidden, and how solid it looks.
                    if let Some((shown, opacity)) = &mut new_display {
                        let toggle = ui
                            .checkbox(shown, "Show the rest of the assembly")
                            .on_hover_text("Shown for reference; it updates as you edit. Hidden, it is neither drawn nor picked");
                        hits["display:show"] = serde_json::json!([toggle.rect.min.x, toggle.rect.min.y, toggle.rect.width(), toggle.rect.height()]);
                        ui.add_enabled_ui(*shown, |ui| {
                            let slider = ui
                                .add(eframe::egui::Slider::new(opacity, 0.05..=1.0).show_value(false))
                                .on_hover_text("How solid the rest of the assembly looks");
                            hits["display:opacity"] = serde_json::json!([slider.rect.min.x, slider.rect.min.y, slider.rect.width(), slider.rect.height()]);
                        });
                    }
                });
                // A face of another part selected: what it can become in this
                // part (copied, never linked).
                if let Some(part) = &context_part {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(format!("Face of {part}:"));
                        let button = ui.button("Sketch on this face").on_hover_text(
                            "A datum on this face (copied, not linked: it stays if that part moves), and a new sketch on it",
                        );
                        hits["context-sketch"] = serde_json::json!([button.rect.min.x, button.rect.min.y, button.rect.width(), button.rect.height()]);
                        if button.clicked() {
                            context_action = Some(true);
                        }
                        let button = ui.button("Datum here").on_hover_text(
                            "A datum on this face (copied, not linked: it stays if that part moves)",
                        );
                        hits["context-datum"] = serde_json::json!([button.rect.min.x, button.rect.min.y, button.rect.width(), button.rect.height()]);
                        if button.clicked() {
                            context_action = Some(false);
                        }
                    });
                }
                for problem in &problems {
                    ui.colored_label(ui.visuals().warn_fg_color, format!("Constraint {problem}"));
                }
                if !modified.is_empty() {
                    ui.weak(format!(
                        "Modified with their own files (written when the assembly is saved): {}",
                        modified.join(", ")
                    ));
                }
                if let Some(error) = &error {
                    ui.colored_label(ui.visuals().error_fg_color, format!("Not applied: {error}"));
                }
                if let Some(line) = &checkout {
                    ui.weak(line);
                }
            });
            let escape = ui.input(|i| i.key_pressed(eframe::egui::Key::Escape))
                && ui.ctx().memory(|m| m.focused().is_none())
                && self.work_engine().is_some_and(|engine| {
                    !engine.has_selection()
                        && engine.context_selection().is_none()
                        && !engine.sketch_mode()
                        && !engine.sketch3d_mode()
                        && !engine.ref_select_active()
                });
            if let (Some((shown, opacity)), true) = (new_display, new_display != display) {
                if let Some(engine) = self.work_engine_mut() {
                    engine.set_context_display(shown, opacity);
                }
            }
            if let Some(sketch) = context_action {
                if let Some(engine) = self.work_engine_mut() {
                    let made = if sketch { engine.sketch_on_context() } else { engine.datum_on_context() };
                    if let Err(error) = made {
                        engine.push_notice(format!("Work part: {error}"));
                    }
                }
            }
            if finish || escape {
                if let Err(error) = self.return_to_assembly() {
                    self.engine_mut().push_notice(format!("Work part: {error}"));
                }
            } else if let Some(depth) = up_to {
                if let Err(error) = self.work_part_up_to(depth) {
                    self.engine_mut().push_notice(format!("Work part: {error}"));
                }
            }
        }
        if modal {
            if let Some(hits) = hits.as_object_mut() {
                hits.retain(|key, _| key.starts_with("checkout:"));
            }
        }
        if let Some(state) = state {
            crate::automation::registry::publish(
                "__brepWorkPart",
                "active Work part session: component path, part path, mode (inPlace|newTab), context render state, assembly context solids (assembly coordinates), pushes, undo coupling, constraint problems; null when inactive",
                &state.to_string(),
            );
            crate::automation::registry::publish(
                "__brepWorkPartHit",
                "Work part controls: return, crumb:<depth> (the breadcrumb: 0 the assembly, then each sub-assembly above the part), up (nested parts), and checkout:yes / checkout:no while the checkout offer is shown",
                &hits.to_string(),
            );
        }
    }
}

/// The assembly's parts-library entries → their `sourceSignature`.
fn library_signatures(document: &Value) -> std::collections::BTreeMap<String, String> {
    document["partsLibrary"]
        .as_object()
        .into_iter()
        .flatten()
        .map(|(name, entry)| (name.clone(), entry["sourceSignature"].as_str().unwrap_or_default().to_string()))
        .collect()
}

/// Refresh the selected definition and any copies with the same source identity.
/// Invalidate every changed ancestor snapshot so nested assemblies rebuild too.
pub(crate) fn refresh_parts(
    owner: &mut Value,
    path: &[String],
    source: Option<&str>,
    edited: &Value,
    notes: &mut Vec<String>,
) -> Result<bool, String> {
    let names: Vec<String> = owner["partsLibrary"]
        .as_object()
        .map(|entries| entries.keys().cloned().collect())
        .unwrap_or_default();
    let mut changed = false;
    for name in names {
        let entry = &owner["partsLibrary"][&name];
        let on_path = path.first() == Some(&name);
        let target = (on_path && path.len() == 1)
            || source.is_some_and(|s| entry["sourceKey"].as_str() == Some(s));
        let tail = if on_path { &path[1..] } else { &[] };
        // A definition that holds no copy of the part is left as it is, without
        // the copy of it a refresh would make and throw away (on l500 most of
        // each module, at every depth).
        if !target && !holds_part(&entry["document"], tail, source) {
            continue;
        }
        let mut child = entry["document"].clone();
        let updated = if target {
            child = edited.clone();
            true
        } else {
            refresh_parts(&mut child, tail, source, edited, notes)?
        };
        if updated {
            notes.extend(crate::panels::parts_library::refresh_library_entry_value(owner, &name, child)?);
            changed = true;
        }
    }
    Ok(changed)
}

/// Whether `owner`'s library holds, at any depth, an entry [`refresh_parts`]
/// would replace: the one at the end of `path`, or one with `source`.
fn holds_part(owner: &Value, path: &[String], source: Option<&str>) -> bool {
    owner["partsLibrary"].as_object().into_iter().flatten().any(|(name, entry)| {
        let on_path = path.first() == Some(name);
        (on_path && path.len() == 1)
            || source.is_some_and(|s| entry["sourceKey"].as_str() == Some(s))
            || holds_part(&entry["document"], if on_path { &path[1..] } else { &[] }, source)
    })
}

fn component_pose(document: &Value, transform: &Value) -> Result<Matrix, String> {
    let read = |field: &str| -> Result<[f64; 3], String> {
        let mut out = [0.; 3];
        for (i, key) in ["x", "y", "z"].iter().enumerate() {
            let value = if transform[field].is_object() {
                &transform[field][*key]
            } else {
                &transform[field][i]
            };
            out[i] = match value {
                Value::Null => 0.,
                Value::Number(n) => n.as_f64().ok_or("invalid component pose")?,
                Value::String(s) => brep_kernel::eval_expression(
                    document["expressions"].as_str().unwrap_or(""),
                    &document["configurator"],
                    s,
                )?,
                _ => return Err("invalid component pose".into()),
            };
            if !out[i].is_finite() {
                return Err("non-finite component pose".into());
            }
        }
        Ok(out)
    };
    let t = read("translate")?;
    let r = read("rotateEulerDeg")?.map(f64::to_radians);
    let (a, b) = (r[0].cos(), r[0].sin());
    let (c, d) = (r[1].cos(), r[1].sin());
    let (e, f) = (r[2].cos(), r[2].sin());
    Ok([
        c * e,
        -c * f,
        d,
        t[0],
        a * f + b * e * d,
        a * e - b * f * d,
        -b * c,
        t[1],
        b * f - a * e * d,
        b * e + a * f * d,
        a * c,
        t[2],
        0.,
        0.,
        0.,
        1.,
    ])
}
fn multiply(a: Matrix, b: Matrix) -> Matrix {
    let mut out = [0.; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = (0..4).map(|k| a[r * 4 + k] * b[k * 4 + c]).sum();
        }
    }
    out
}
fn inverse_rigid(m: Matrix) -> Matrix {
    let mut out = IDENTITY;
    for r in 0..3 {
        for c in 0..3 {
            out[r * 4 + c] = m[c * 4 + r];
        }
        out[r * 4 + 3] = -(0..3).map(|k| m[k * 4 + r] * m[k * 4 + 3]).sum::<f64>();
    }
    out
}
fn vector_at(m: Matrix, p: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|r| (0..3).map(|k| m[r * 4 + k] * p[k]).sum())
}
fn point_at(m: Matrix, p: [f64; 3]) -> [f64; 3] {
    let v = vector_at(m, p);
    std::array::from_fn(|r| v[r] + m[r * 4 + 3])
}

pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] =
    &[crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "return",
        meaning: "return to assembly and apply Work part edits",
        command: None,
    },
    crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "checkout:",
        meaning: "the offer to check out a PLM part made the Work part: checkout:yes (default), checkout:no",
        command: None,
    },
    crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "context-",
        meaning: "a face of another part selected (context): context-sketch (a datum on it, copied not linked, and a new sketch there), context-datum (the datum only)",
        command: None,
    },
    crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "display:",
        meaning: "the rest of the assembly (context): display:show (show/hide it), display:opacity (its opacity slider)",
        command: None,
    },
    crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "crumb:",
        meaning: "the Work part breadcrumb: crumb:0 returns to the assembly, crumb:<k> makes the k-th sub-assembly above the part the Work part",
        command: None,
    },
    crate::automation::hit_keys::HitKeyDoc {
        panel: "workpart",
        prefix: "up",
        meaning: "a nested Work part: make the sub-assembly holding it the Work part",
        command: None,
    }];

