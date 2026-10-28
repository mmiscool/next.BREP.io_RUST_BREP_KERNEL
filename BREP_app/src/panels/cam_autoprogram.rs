//! Preview automatic programming before making one undoable document edit.
//! Exact topology arrives through the existing runner request, including in
//! the browser. The preview is invalidated when its document or bodies change.

use crate::panels::{cam, cam_setup};
use crate::document::cam::CamDocument;
use brep_cam::autoprogram::{self, AutoInput, Plan, PlannedOperation};
use brep_cam::features::{Recognized, RecognitionInput, TechDb};
use brep_cam::kernel_adapter::{BodyInput, machine_snapshot, revolved_faces};
use brep_cam::kernel_adapter::features::face_carriers;
use brep_cam::kernel_adapter::planar::horizontal_faces;
use brep_cam::library::LibraryUnits;
use brep_cam::feeds::WorkMaterial;
use brep_cam::model::{Setup, Tool, ToolGeometry, ToolKind};
use brep_cam::{NullSink, Runner, Tolerance};
use eframe::egui;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use brep_render::engine_state::EngineState;

/// Transient preview; no operation or fallback tool is saved until Create.
#[derive(Default)]
pub struct AutoProgramPanel {
    pending: Option<String>,
    preview: Option<Preview>,
    error: Option<String>,
    pub open: bool,
}


struct Preview {
    setup: String,
    enabled: bool,
    additional: Vec<SetupPreview>,
    unreached: Vec<autoprogram::Skipped>,
    document: CamDocument,
    revisions: Vec<(String, u64)>,
    recognized: Recognized,
    plan: Plan,
    tools: Vec<Tool>,
    fallback: BTreeSet<String>,
    selected: BTreeSet<String>,
}

struct SetupPreview {
    orientation: crate::document::cam::SetupOrientation,
    proposal: autoprogram::multi::SetupPlan,
    enabled: bool,
    selected: BTreeSet<String>,
}

fn body_names(setup: &Setup) -> BTreeSet<String> {
    let mut names: BTreeSet<_> = setup.targets.iter().chain(&setup.protected).map(|r| r.solid.clone()).collect();
    if let brep_cam::model::Stock::FromSolid { solid } = &setup.stock { names.insert(solid.clone()); }
    names
}

fn revisions(state: &EngineState, setup: &Setup) -> Result<Vec<(String, u64)>, String> {
    body_names(setup).into_iter().map(|name| {
        let revision = state.scene.solid(&name).ok_or_else(|| format!("body `{name}` is not on screen"))?.revision;
        Ok((name, revision))
    }).collect()
}

/// Conservative starter tools paired with the builtin technology rules.
/// These are proposals, clearly labelled, never claims about the machine's
/// physical magazine. Only tools used by selected operations are imported.
fn fallback_tools(document: &CamDocument) -> Result<Vec<Tool>, String> {
    let mut tools = Vec::new();
    let mut number = document.tools.iter().map(|t| t.number).max().unwrap_or(0);
    for (kind, diameter, length) in [
        (ToolKind::FaceMill, 40., 5.),
        (ToolKind::Flat, 10., 30.), (ToolKind::Flat, 8., 30.),
        (ToolKind::Flat, 6., 30.), (ToolKind::Flat, 4., 25.), (ToolKind::Flat, 2., 20.),
        (ToolKind::Drill, 3., 40.), (ToolKind::Drill, 4., 40.),
        (ToolKind::Drill, 6., 40.), (ToolKind::Drill, 8., 40.), (ToolKind::Drill, 10., 40.),
        (ToolKind::SpotDrill, 10., 15.), (ToolKind::Chamfer, 10., 15.),
    ] {
        number = number.checked_add(1).ok_or("tool numbers are exhausted")?;
        let mut id = format!("auto-builtin-{number}");
        while document.tool(&id).is_some() { id.push('_'); }
        let mut tool = Tool::new(id, number, kind, ToolGeometry::flat(diameter, length));
        tool.name = format!("Builtin {diameter} mm {kind:?}");
        tool.description = "Automatic programming fallback; verify the available physical tool before machining".into();
        tool.material = "carbide".into();
        tool.geometry.flute_count = Some(if matches!(kind, ToolKind::Drill | ToolKind::SpotDrill) { 2 } else { 3 });
        if matches!(kind, ToolKind::Drill | ToolKind::SpotDrill | ToolKind::Chamfer) {
            tool.geometry.included_angle_deg = Some(if kind == ToolKind::Drill { 118. } else { 90. });
        }
        brep_cam::library::validate_tool(&tool)?;
        tools.push(tool);
    }
    Ok(tools)
}

fn phase(role: &str) -> u8 {
    match role { "face" => 0, "rough" => 1, "finish" | "chamfer" => 2, "spot" => 3, "drill" => 4, "tap" => 5, _ => 6 }
}

/// Try document tools for EACH rule first. A missing tool for one rule
/// cannot replace a usable document tool chosen for a different rule.
fn plan_with_fallback(input: &AutoInput<'_>, fallback: &[Tool]) -> Result<Plan, String> {
    let mut combined = input.tools.to_vec();
    combined.extend_from_slice(fallback);
    let mut operations = Vec::<PlannedOperation>::new();
    let mut reasons = BTreeMap::<String, Vec<String>>::new();
    for rule in &input.techdb.rules {
        for operation in &rule.operations {
            let mut db = input.techdb.clone();
            db.rules = vec![brep_cam::features::techdb::Rule { feature: rule.feature.clone(), operations: vec![operation.clone()] }];
            let first = autoprogram::plan(&AutoInput { techdb: &db, ..*input }).map_err(|e| e.to_string())?;
            let second = autoprogram::plan(&AutoInput { techdb: &db, tools: &combined, ..*input }).map_err(|e| e.to_string())?;
            for feature in input.recognized.features.iter().filter(|f| f.kind.name() == rule.feature) {
                let own: Vec<_> = first.operations.iter().filter(|p| p.feature == feature.id).cloned().collect();
                let chosen: Vec<_> = if own.is_empty() { second.operations.iter().filter(|p| p.feature == feature.id).cloned().collect() } else { own };
                if chosen.is_empty() {
                    if let Some(reason) = second.skipped.iter().find(|s| s.feature == feature.id) {
                        reasons.entry(feature.id.clone()).or_default().push(reason.reason.clone());
                    }
                }
                operations.extend(chosen);
            }
        }
    }
    let mut skipped = Vec::new();
    for f in &input.recognized.features {
        let why = reasons.remove(&f.id).unwrap_or_default();
        if !operations.iter().any(|p| p.feature == f.id) {
            skipped.push(autoprogram::Skipped { feature: f.id.clone(), kind: f.kind.name().into(), reason: if why.is_empty() {
                f.reason.clone().unwrap_or_else(|| format!("no builtin technology rule for {}", f.kind.name()))
            } else { why.join("; ") } });
        } else {
            for p in operations.iter_mut().filter(|p| p.feature == f.id) { p.notes.extend(why.clone()); }
        }
    }
    let tool = |p: &PlannedOperation| combined.iter().find(|t| t.id == p.tool).expect("planner selected an input tool");
    let index = |p: &PlannedOperation| input.recognized.features.iter().position(|f| f.id == p.feature).unwrap_or(usize::MAX);
    operations.sort_by(|a, b| phase(&a.role).cmp(&phase(&b.role))
        .then_with(|| if phase(&a.role) == 1 { tool(b).geometry.diameter.total_cmp(&tool(a).geometry.diameter) } else { std::cmp::Ordering::Equal })
        .then(tool(a).number.cmp(&tool(b).number)).then(a.tool.as_str().cmp(b.tool.as_str())).then(index(a).cmp(&index(b))));
    let mut ordered = Vec::<PlannedOperation>::new();
    for ph in 0..=6 {
        let group: Vec<_> = operations.iter().filter(|p| phase(&p.role) == ph).cloned().collect();
        if ph != 1 {
            if let Some(last) = ordered.last() {
                let (same, rest): (Vec<_>, Vec<_>) = group.into_iter().partition(|p| p.tool == last.tool);
                ordered.extend(same); ordered.extend(rest);
                continue;
            }
        }
        ordered.extend(group);
    }
    let tool_changes = ordered.windows(2).filter(|w| w[0].tool != w[1].tool).count() as u32;
    Ok(Plan { operations: ordered, skipped, tool_changes })
}

fn recognize_setup(state: &EngineState, setup: &Setup) -> Result<(Recognized, brep_cam::MeshSnapshot), String> {
    let tolerance = Tolerance::default();
    let chord = 0.01;
    let (mesh, unresolved) = machine_snapshot(setup, chord, tolerance, |name| state.scene.exact_solid(name)).map_err(|e| e.to_string())?;
    if !unresolved.is_empty() { return Err(format!("exact topology unavailable for {}", unresolved.join(", "))); }
    let mut carriers = Vec::new();
    let mut regions = Vec::new();
    let mut revolved = Vec::new();
    for name in setup.targets.iter().map(|r| r.solid.as_str()).collect::<BTreeSet<_>>() {
        let solid = state.scene.exact_solid(name).ok_or_else(|| format!("exact topology unavailable for `{name}`"))?;
        let body = BodyInput { name, solid, protected_only: false };
        carriers.extend(face_carriers(&body, setup, &tolerance));
        regions.extend(horizontal_faces(solid, &setup.model_to_machine, chord, &tolerance));
        revolved.extend(revolved_faces(&body, setup));
    }
    let mut sink = NullSink;
    let mut runner = Runner::detached(&mut sink);
    let holes = brep_cam::holes::recognize(&revolved, &mesh, tolerance, &mut runner).map_err(|e| e.to_string())?;
    let recognized = brep_cam::features::recognize(&RecognitionInput { mesh: &mesh, carriers: &carriers, regions: &regions, holes: &holes, tolerance }).map_err(|e| e.to_string())?;
    Ok((recognized, mesh))
}

fn build_preview(state: &EngineState, doc: CamDocument, setup_id: &str) -> Result<Preview, String> {
    let setup = doc.setup(setup_id).ok_or_else(|| format!("no setup `{setup_id}`"))?;
    if setup.targets.is_empty() { return Err("select the setup's target bodies before programming".into()); }
    let revisions = revisions(state, setup)?;
    let (recognized, first_mesh) = recognize_setup(state, setup)?;
    let stock_top = cam_setup::stock_bounds(setup, &state.scene).map(|(min, max)| {
        (0..8).map(|i| setup.model_to_machine.apply_point(brep_cam::model::Point3::new(
            if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z })).z)
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let db = TechDb::builtin();
    let machine = doc.setup_machine(setup_id);
    if setup.machine.is_some() && machine.is_none() { return Err("the setup's selected machine is missing".into()); }
    let extra = fallback_tools(&doc)?;
    let input = AutoInput { recognized: &recognized, setup, tools: &doc.tools,
        machine: machine.as_ref(), material: WorkMaterial::Aluminium, techdb: &db, units: LibraryUnits::Millimeters, stock_top };
    let mut plan = plan_with_fallback(&input, &extra)?;
    autoprogram::access::constrain_open_extensions(&mut plan, &recognized, &first_mesh, &Tolerance::default()).map_err(|e| e.to_string())?;
    let axes = autoprogram::multi::needed_axes(&recognized);
    let mut candidates = Vec::new();
    let mut orientations = BTreeMap::new();
    let mut candidate_meshes = BTreeMap::new();
    if !axes.is_empty() {
        let (min, max) = cam_setup::stock_bounds(setup, &state.scene)
            .ok_or("additional setups need a resolved initial stock boundary; regenerate/review the carried setup first")?;
        let bounds = brep_cam::model::Bounds3 { min, max };
        for axis in axes {
            let mut proposed = autoprogram::multi::candidate_setup(setup, axis, bounds).map_err(|e| e.to_string())?;
            let source = recognized.features.iter().find(|f| !f.machinable && f.direction.dot(axis.vector()) > 1. - 1e-6).ok_or("approach feature disappeared")?;
            let face = source.faces.first().ok_or("approach feature has no persistent face")?;
            let origin = setup.machine_to_model().apply_point(brep_cam::model::Point3::ZERO);
            let mut orientation = crate::document::cam::SetupOrientation { z_ref: Some(face.clone()),
                origin_ref: Some(format!("auto@{},{},{}", origin.x, origin.y, origin.z)), ..Default::default() };
            let mut temporary = doc.clone();
            temporary.setups.push(proposed.clone());
            temporary.set_orientation(proposed.id.as_str(), orientation.clone());
            let frame = cam::setup_frame(state, &temporary, proposed.id.as_str())?;
            let expected = setup.machine_to_model().apply_vector(axis.vector());
            orientation.flip_z = frame.z_axis.dot(expected) < 0.;
            temporary.set_orientation(proposed.id.as_str(), orientation.clone());
            proposed.model_to_machine = cam::setup_frame(state, &temporary, proposed.id.as_str())?.transform()?;
            let top = proposed.model_to_machine.apply_bounds(&bounds).max.z;
            proposed.heights.clearance = top + 20.; proposed.heights.retract = top + 10.; proposed.heights.feed = top + 5.;
            orientations.insert(proposed.id.to_string(), orientation);
            let (recognized, mesh) = recognize_setup(state, &proposed)?;
            candidate_meshes.insert(proposed.id.to_string(), mesh);
            candidates.push(autoprogram::multi::Candidate { axis,
                stock_top: Some(proposed.model_to_machine.apply_bounds(&bounds).max.z), setup: proposed, recognized });
        }
    }
    let mut multi = autoprogram::multi::plan_candidates(&input, plan, &candidates, |input| {
        let mut plan = plan_with_fallback(input, &extra).map_err(brep_cam::CamError::InvalidInput)?;
        let mesh = candidate_meshes.get(input.setup.id.as_str()).ok_or_else(|| brep_cam::CamError::InvalidInput("candidate mesh disappeared".into()))?;
        autoprogram::access::constrain_open_extensions(&mut plan, input.recognized, mesh, &Tolerance::default())?;
        Ok(plan)
    }).map_err(|e| e.to_string())?;
    let first = multi.setups.remove(0);
    let plan = first.plan;
    let selected = plan.operations.iter().map(|p| p.feature.clone()).collect();
    let additional = multi.setups.into_iter().map(|proposal| SetupPreview {
        orientation: orientations.remove(proposal.setup.id.as_str()).expect("candidate orientation"),
        selected: proposal.plan.operations.iter().map(|p| p.feature.clone()).collect(), proposal, enabled: true,
    }).collect();
    let fallback = extra.iter().map(|t| t.id.to_string()).collect();
    let mut tools = doc.tools.clone(); tools.extend(extra);
    Ok(Preview { setup: setup_id.into(), enabled: true, additional, unreached: multi.unreached,
        document: doc, revisions, recognized, plan, tools, fallback, selected })
}

impl AutoProgramPanel {
    pub fn preview(&mut self, state: &mut EngineState, setup: &str) -> Result<(), String> {
        if let Some(why) = cam::sync_setup_frame(state, setup) { return Err(why); }
        let doc = cam::cam_document(state)?;
        let setup_record = doc.setup(setup).ok_or_else(|| format!("no setup `{setup}`"))?;
        let names = body_names(setup_record);
        revisions(state, setup_record)?;
        state.request_exact_solids(names.iter().map(String::as_str));
        self.preview = None;
        self.error = None;
        self.pending = Some(setup.into());
        self.open = true;
        self.poll(state);
        Ok(())
    }

    pub fn poll(&mut self, state: &mut EngineState) {
        let Some(id) = self.pending.clone() else { return };
        state.pump_topology();
        if state.topology_pending() { return; }
        self.pending = None;
        match cam::cam_document(state).and_then(|doc| build_preview(state, doc, &id)) {
            Ok(preview) => self.preview = Some(preview),
            Err(why) => self.error = Some(why),
        }
    }

    pub fn select(&mut self, feature: &str, selected: bool) -> Result<(), String> {
        let p = self.preview.as_mut().ok_or("preview the part first")?;
        let (recognized, plan, selection) = if p.recognized.features.iter().any(|f| f.id == feature) {
            (&p.recognized, &p.plan, &mut p.selected)
        } else if let Some(s) = p.additional.iter_mut().find(|s| s.proposal.recognized.features.iter().any(|f| f.id == feature)) {
            (&s.proposal.recognized, &s.proposal.plan, &mut s.selected)
        } else { return Err(format!("no recognized feature `{feature}`")); };
        let _ = recognized;
        if selected && !plan.operations.iter().any(|p| p.feature == feature) { return Err(format!("feature `{feature}` has no planned operations")); }
        if selected { selection.insert(feature.into()); } else { selection.remove(feature); }
        Ok(())
    }

    pub fn select_setup(&mut self, setup: &str, selected: bool) -> Result<(), String> {
        let p = self.preview.as_mut().ok_or("preview the part first")?;
        if p.setup == setup { p.enabled = selected; }
        else if let Some(s) = p.additional.iter_mut().find(|s| s.proposal.setup.id.as_str() == setup) { s.enabled = selected; }
        else { return Err(format!("no proposed setup `{setup}`")); }
        Ok(())
    }

    fn current<'a>(&'a self, state: &EngineState) -> Result<&'a Preview, String> {
        let preview = self.preview.as_ref().ok_or("preview the part first")?;
        let doc = cam::cam_document(state)?;
        if doc != preview.document || revisions(state, doc.setup(&preview.setup).ok_or("setup was removed")?)? != preview.revisions {
            return Err("the model or CAM inputs changed; preview the part again".into());
        }
        Ok(preview)
    }

    pub fn create(&mut self, state: &mut EngineState) -> Result<Vec<String>, String> {
        let p = self.current(state)?;
        let mut doc = p.document.clone();
        let first = doc.setup(&p.setup).ok_or("setup was removed")?.clone();
        let mut groups = Vec::new();
        if p.enabled { groups.push((None, first.clone(), &p.recognized, &p.plan, &p.selected)); }
        for s in p.additional.iter().filter(|s| s.enabled) {
            groups.push((Some(s.orientation.clone()), s.proposal.setup.clone(), &s.proposal.recognized, &s.proposal.plan, &s.selected));
        }
        if !groups.iter().any(|(_,_,_,plan,selection)| plan.operations.iter().any(|op| selection.contains(&op.feature))) {
            return Err("select at least one programmable feature".into());
        }
        let fingerprints: BTreeMap<_, _> = body_names(&first).into_iter().map(|name| {
            let (_, hash) = state.scene.exact_solid_fingerprinted(&name).ok_or_else(|| format!("exact body `{name}` changed; preview again"))?;
            Ok((name, hash))
        }).collect::<Result<_, String>>()?;
        let mut tools = BTreeMap::<String, String>::new();
        for (_, _, _, plan, selection) in &groups {
            for planned in plan.operations.iter().filter(|op| selection.contains(&op.feature)) {
                let id = planned.tool.to_string();
                if tools.contains_key(&id) { continue; }
                if !p.fallback.contains(&id) { tools.insert(id.clone(), id); continue; }
                let mut tool = p.tools.iter().find(|t| t.id == planned.tool).ok_or("planned tool disappeared")?.clone();
                let new_id = doc.next_id("T");
                tool.id = new_id.as_str().into();
                tool.number = doc.tools.iter().map(|t| t.number).max().unwrap_or(0).checked_add(1).ok_or("tool numbers are exhausted")?;
                if let Some(holder) = &mut tool.holder { holder.id = format!("{new_id}-holder").into(); }
                doc.tools.push(tool); tools.insert(id, new_id);
            }
        }
        let mut created = Vec::new();
        let mut previous = first.id.clone();
        for (orientation, mut setup, recognized, plan, selection) in groups {
            let selected: Vec<_> = plan.operations.iter().filter(|op| selection.contains(&op.feature)).collect();
            if selected.is_empty() { continue; }
            if let Some(orientation) = orientation {
                setup.id = doc.next_id("SETUP").into();
                // Unticked/empty proposals are absent from the actual stock chain.
                setup.stock = brep_cam::model::Stock::FromRest { setup: previous.clone() };
                doc.set_orientation(setup.id.as_str(), orientation);
                doc.setups.push(setup.clone());
            }
            for planned in selected {
                let mut planned = planned.clone();
                let id = doc.next_id("OP");
                planned.operation.id = id.as_str().into();
                planned.operation.setup = setup.id.clone();
                planned.operation.tool = tools[planned.tool.as_str()].as_str().into();
                planned.tool = planned.operation.tool.clone();
                let tool = doc.tool(planned.tool.as_str()).ok_or("planned tool disappeared")?;
                let feature = recognized.features.iter().find(|f| f.id == planned.feature).ok_or("planned feature disappeared")?;
                let binding = brep_cam::autoprogram::binding::Binding::new(&planned, feature, &setup, tool, fingerprints.clone());
                doc.autoprogram_inputs.insert(id.clone(), binding);
                doc.operations.push(planned.operation); created.push(id);
            }
            previous = setup.id;
        }
        cam::write_cam_document(state, &doc, None);
        self.close();
        Ok(created)
    }

    pub fn state(&self, state: &EngineState) -> Value {
        let programmed: Vec<_> = cam::cam_document(state).ok().map(|doc| doc.operations.iter().filter_map(|op| {
            let binding = doc.autoprogram_inputs.get(op.id.as_str())?;
            let path = state.cam.results.get(op.id.as_str()).and_then(|r| r.toolpath.as_ref());
            let minimum_cut_z = path.and_then(|path| path.motions.iter().filter_map(|m| match &m.kind {
                brep_cam::model::MotionKind::Cut { to } | brep_cam::model::MotionKind::ArcCw { to, .. } | brep_cam::model::MotionKind::ArcCcw { to, .. } => Some(to.z), _ => None,
            }).reduce(f64::min));
            Some(json!({ "id": op.id, "setup": op.setup, "feature": binding.feature.id, "kind": binding.feature.kind,
                "status": crate::panels::cam_generate::status_of(state, &doc, op.id.as_str()).word(), "minimumCutZ": minimum_cut_z,
                "warnings": path.map(|p| &p.report.warnings), "reRecognised": path.is_some_and(|p| p.report.warnings.iter().any(|w| w.code == "autoprogram-re-recognised")) }))
        }).collect()).unwrap_or_default();
        let all_programmed_ok = !programmed.is_empty() && programmed.iter().all(|op| op["status"] == "ok");
        let Some(p) = &self.preview else { return json!({ "open": self.open, "busy": self.pending.is_some(), "error": self.error, "features": [], "operations": [], "allProgrammedOk": all_programmed_ok, "programmed": programmed }); };
        let feature_rows = |recognized: &Recognized, plan: &Plan, selected: &BTreeSet<String>| recognized.features.iter().map(|f| json!({
            "id": f.id, "kind": f.kind.name(), "solid": f.solid, "selected": selected.contains(&f.id),
            "operations": plan.operations.iter().filter(|op| op.feature == f.id).map(|op| json!({
                "id": op.operation.id, "name": op.operation.name, "strategy": op.operation.strategy.kind_name(),
                "role": op.role, "tool": op.tool, "toolName": p.tools.iter().find(|t| t.id == op.tool).map(|t| &t.name),
                "toolSource": if p.fallback.contains(op.tool.as_str()) { "builtin" } else { "document" }, "notes": op.notes
            })).collect::<Vec<_>>(),
            "refusals": plan.skipped.iter().filter(|s| s.feature == f.id).map(|s| &s.reason).collect::<Vec<_>>()
        })).collect::<Vec<_>>();
        let features = feature_rows(&p.recognized, &p.plan, &p.selected);
        let first = p.document.setup(&p.setup).expect("preview setup");
        let mut setups = vec![json!({ "id": p.setup, "name": first.name, "axis": "+Z", "selected": p.enabled,
            "additional": false, "orientation": first.model_to_machine, "stock": first.stock,
            "features": features, "operationCount": if p.enabled { p.plan.operations.iter().filter(|op| p.selected.contains(&op.feature)).count() } else { 0 } })];
        let mut previous = p.setup.clone();
        for proposed in &p.additional {
            let proposal = &proposed.proposal;
            let count = if proposed.enabled { proposal.plan.operations.iter().filter(|op| proposed.selected.contains(&op.feature)).count() } else { 0 };
            setups.push(json!({ "id": proposal.setup.id, "name": proposal.setup.name, "axis": proposal.axis.map(|a| a.name()),
                "selected": proposed.enabled, "additional": true, "orientation": proposal.setup.model_to_machine,
                "stock": brep_cam::model::Stock::FromRest { setup: previous.as_str().into() }, "covered": proposal.covered,
                "features": feature_rows(&proposal.recognized, &proposal.plan, &proposed.selected), "operationCount": count }));
            if count > 0 { previous = proposal.setup.id.to_string(); }
        }
        let operation_count: u64 = setups.iter().filter_map(|s| s["operationCount"].as_u64()).sum();

        json!({ "open": self.open, "busy": false, "error": self.error, "stale": self.current(state).is_err(), "allProgrammedOk": all_programmed_ok, "programmed": programmed,
            "setup": p.setup, "setups": setups, "setupCount": setups.len(), "unreached": p.unreached, "features": features, "featureCount": features.len(),
            "selectedCount": p.selected.len(), "operationCount": operation_count,
            "unassignedFaces": p.recognized.unassigned, "material": "aluminium",
            "notice": "Document tools are preferred. Tools labelled Builtin are proposed fallback tools. Cutting data assumes aluminium." })
    }

    pub fn close(&mut self) { self.open = false; self.pending = None; self.preview = None; self.error = None; }

    /// The caller owns the small shared pane integration; all preview rows
    /// and their stable hit keys are owned here.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, hits: &mut HashMap<String, egui::Rect>) {
        self.poll(state);
        if !self.open { return; }
        ui.separator();
        ui.heading("Program part automatically");
        ui.label("Preview every feature before creating operations. Builtin tools are proposed only where document tools do not fit. Cutting data: aluminium.");
        if self.pending.is_some() { ui.spinner(); ui.label("Reading exact part geometry…"); }
        if let Some(why) = &self.error { ui.colored_label(egui::Color32::RED, why); }
        if let Some(p) = &mut self.preview {
            ui.label("Select setups before creating operations. Later setups continue from the preceding selected setup's stock.");
            let mut groups = vec![(p.setup.as_str(), "Current setup", &mut p.enabled, &p.recognized, &p.plan, &mut p.selected)];
            for extra in &mut p.additional {
                groups.push((extra.proposal.setup.id.as_str(), extra.proposal.setup.name.as_str(), &mut extra.enabled,
                    &extra.proposal.recognized, &extra.proposal.plan, &mut extra.selected));
            }
            for (id, name, enabled, recognized, plan, selected) in groups {
                let response = ui.checkbox(enabled, name);
                hits.insert(format!("cam:auto:setup:{id}"), response.rect);
                if id != p.setup { ui.small("Stock: result of the preceding selected setup. Refixture and verify the work offset before running this setup."); }
                ui.add_enabled_ui(*enabled, |ui| {
                    for feature in &recognized.features {
                        let ops: Vec<_> = plan.operations.iter().filter(|op| op.feature == feature.id).collect();
                        let mut checked = selected.contains(&feature.id);
                        let response = ui.add_enabled(!ops.is_empty(), egui::Checkbox::new(&mut checked, format!("{} · {} · {}", feature.id, feature.kind.name(), feature.solid)));
                        hits.insert(format!("cam:auto:feature:{}", feature.id), response.rect);
                        if response.changed() { if checked { selected.insert(feature.id.clone()); } else { selected.remove(&feature.id); } }
                        for op in ops {
                            let tool = p.tools.iter().find(|t| t.id == op.tool).expect("planned tool");
                            ui.label(format!("{} — {} ({})", op.operation.name, tool.name, if p.fallback.contains(op.tool.as_str()) { "Builtin fallback" } else { "Document tool" }));
                            for note in &op.notes { ui.small(note); }
                        }
                        for refusal in plan.skipped.iter().filter(|s| s.feature == feature.id) { ui.label(format!("Not programmed here: {}", refusal.reason)); }
                    }
                });
            }
            for refusal in &p.unreached { ui.label(format!("Not reached: {}", refusal.reason)); }
        }
        if let Err(why) = self.current(state) {
            if self.preview.is_some() { ui.colored_label(egui::Color32::YELLOW, why); }
        }
        ui.horizontal(|ui| {
            let create = ui.add_enabled(self.current(state).is_ok(), egui::Button::new("Create selected operations"));
            hits.insert("cam:auto:create".into(), create.rect);
            if create.clicked() { if let Err(why) = self.create(state) { self.error = Some(why); } }
            let close = ui.button("Close");
            hits.insert("cam:auto:close".into(), close.rect);
            if close.clicked() { self.close(); }
        });
    }
}

#[cfg(feature = "automation")]
#[derive(serde::Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CommandArgs {
    /// preview, state, select, selectSetup, create or close.
    pub action: String,
    #[serde(default)]
    pub setup: Option<String>,
    #[serde(default)]
    pub feature: Option<String>,
    #[serde(default)]
    pub selected: Option<bool>,
}

#[cfg(feature = "automation")]
pub fn command(ctx: &mut crate::automation::command::Ctx<'_>, args: Value) -> Result<crate::automation::command::Outcome, String> {
    use crate::automation::command::{parse_args, Outcome};
    let args: CommandArgs = parse_args(args)?;
    ctx.app.cam.autoprogram.poll(ctx.app.docs.engine_mut());
    match args.action.as_str() {
        "preview" => {
            let setup = args.setup.ok_or("preview requires setup")?;
            ctx.app.cam.set_open(None);
            ctx.app.cam.autoprogram.preview(ctx.app.docs.engine_mut(), &setup)?;
        }
        "select" => ctx.app.cam.autoprogram.select(&args.feature.ok_or("select requires feature")?, args.selected.ok_or("select requires selected")?)?,
        "selectSetup" => ctx.app.cam.autoprogram.select_setup(&args.setup.ok_or("selectSetup requires setup")?, args.selected.ok_or("selectSetup requires selected")?)?,
        "create" => return Ok(Outcome::Done(json!({ "created": ctx.app.cam.autoprogram.create(ctx.app.docs.engine_mut())? }))),
        "close" => ctx.app.cam.autoprogram.close(),
        "state" => {},
        _ => return Err("action must be preview, state, select, selectSetup, create or close".into()),
    }
    Ok(Outcome::Done(ctx.app.cam.autoprogram.state(ctx.app.docs.engine())))
}
