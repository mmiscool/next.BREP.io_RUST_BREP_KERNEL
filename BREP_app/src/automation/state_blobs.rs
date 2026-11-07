//! The state blobs that are plain app state: each is a function of the app's
//! data, produced on demand by `BrepApp::state_blob_json`.
//!
//! The frame publishes them into the registry (the wasm build mirrors that to
//! `window.*`), and `state_get` asks the app for them directly at dispatch
//! time, so reading one never waits for a frame. Every other blob in the
//! registry is published while the panels draw (widget rects, the viewport
//! rect, focus, the dock), and reading one of those needs a current layout
//! (`automation::queue::Need::Layout`).

/// `(name, doc)` for every blob `BrepApp::state_blob_json` produces.
pub const STATE_BLOBS: &[(&str, &str)] = &[
    ("__brepCamera", "camera state: kind, eye, target, up, near/far, projection block, worldPerPixel"),
    ("__brepSettings", "render and UI settings"),
    ("__brepSolidColors", "per-solid colour overrides"),
    ("__brepHistory", "history listing {step, features:[{index,type,id}]}"),
    ("__brepGizmo", "transform gizmo state"),
    ("__brepFile", "file dialog state (mode, entries, current name)"),
    ("__brepModel", "model signature: solid count, triangle count, per-solid bounds (change detection)"),
    ("__brepReport", "last run report {featureErrors, featureNotes, featureFulfilment, featureRefusals (only when a feature failed with a typed refusal), featureApproximations (only when a successful feature carries a measured approximation: feature id → [{code, body, measured, bar, volume_bound, edges, message, summary}]), unresolved, displayErrors, featureTimings, featureOutputs}"),
    ("__brepRun", "in-flight run {pending, progress:{generation,index,total,featureId,featureType}|null, cancelled}"),
    ("__brepBusy", "the status bar's working indicator {shown, activities:[{kind, label, document, cancellable, elapsed}]} — everything in flight, most important first; shown once work has lasted 0.2 s"),
    ("__brepExpr", "expressions script, its variables and the configurator"),
    ("__brepPlmDoc", "the active document on the PLM: {access:{editable, reason}, plmDocument, revision:{id,label,lifecycle,lockedBy,lockedByMe}, busy, message, offered:[verb key], files:{staged:{name,mediaType,kind,size,target}, revision:[name], part:[name], problem, notice, status, busy}}; null with no PLM session"),
    ("__brepNotices", "queued toast texts"),
    ("__brepToasts", "toast cards {cards: [{text, severity (error|warning|info), count, shown, superseded}], shown, folded}"),
    ("__brepBug", "bug report panel state"),
    ("__brepDiagnostics", "what this session is running on: renderer, adapter, texture ceiling, version, platform"),
    ("__brepPerf", "rolling per-frame timings in ms {frames, window, dt, ui, publish, sync, fit, overlays, draw} each {avg,p95,max}; staleFrames: frames whose render inputs changed without a render (automation check, never reset)"),
    ("__brepWireHarness", "wire harness connections and the last routing report"),
    ("__brepCam", "CAM state {setups: [{id, name, operations: [op id], record}], tools, operations, ncPrograms, idCounter, strategies, orientations, frames: {id: {origin, xAxis, zAxis, frameError, stock}}, selected, open, notice, error}"),
    ("__brepInstructions", "Instructions pane state {selected: the step selected in the pane, notice, error}"),
    ("__brepInstructionsPlayer", "Player pane state {mode: shop|customer, step: the step shown, insertion: how far its parts have come home (null once they are), pending: a move waits for the next frame, checked: {step: [part index]}, serials: {occurrence: serial}, signed: [step], error}"),
    ("__brepSimulation", "Simulation state {available: the document is an assembly a study can drive, block: the simulation block or null, mates: [{id, kind: distance|angle, value, enabled}] a motor can drive, pane: {notice, error}}"),
    ("__brepCamSim", "CAM simulation player state {loaded, speed, stopAtCollision, cell, showDeviation, notice, error, session: {setup, multiAxis, playing, motion, fraction, time, totalSeconds, progress, motions, completed, applied, caughtUp, removedVolume, stockVolume, unsupported, operation, operations, marks, collisionMotions, deviation, stockVersion}}"),
    ("__brepPorts", "the part's declared connection points (the `ports` block) and the ports tail's report of the last run"),
    ("__brepQualify", "Qualify panel state {selected: the connection-point address it has selected}"),
    ("__brepFamily", "Family table pane state {table, expressions, notColumns, selected, editing, problems, columnProblems, values, generateCalls, report, status, preview: {row, partNumber, applied, showing, message, featureErrors} or null, runsReplied}"),
    ("__brepPmi", "PMI block, report, active view, open annotation or view dialog, and the selected annotation or view"),
    ("__brepSheets", "drawing sheets, the open sheet and its projection, the open and the selected sheet object"),
    ("__brepWorkbench", "active workbench id and the available ids"),
    ("__brepSelection", "selection {solids, faces, edges, datums, vertices}; the name arrays are in pick order, oldest first"),
    ("__brepInfoWindows", "open info windows (mass properties, topology, metadata)"),
    ("__brepInterference", "interference check state"),
    ("__brepAutoConstraints", "auto-constraint inference state"),
    ("__brepStepParts", "STEP parts library panel state"),
    ("__brepSelectionFilter", "which entity kinds a viewport click may pick"),
    ("__brepSelectionInspection", "selection measurements displayed in the status bar: {shown,text,pending,lengthUnit,areaUnit,items}; cached kernel face/edge queries and world vertex coordinates"),
    ("__brepFilterButton", "the status bar's compact selection filter {shown, label}: whether the filter is one button (touch or a narrow bar) and its caption"),
    ("__brepContext", "context action bar state (offers for the selection)"),
    ("__brepPickBar", "the focused pick bar {shown, label, count}: a touch or narrow window's picker while a dialog's reference field picks, all other chrome hidden"),
    ("__brepDocuments", "open document tabs {active, tabs:[{title,name,dirty}]}"),
    ("__brepRecovery", "boot-time recovery prompt entries"),
    ("__brepClosing", "closing the app: {closing, closed, pending}: a close waits for the store's writes to land, unsaved work written for recovery first"),
    ("__brepComponentMove", "assembly component move state"),
    ("__brepSketch", "sketch mode state (session, tool, selection, constraints)"),
    ("__brepSketch3d", "3D sketch mode state: {sketch3dMode, featureId, dof, status, conflicting, tool, workPlane, workPlaneAnchor, pointCount, geometryCount, constraintCount, pendingLen, selection, applicableConstraints, dimensions, externalRefs, lastHit, canUndo, canRedo, entities}"),
    ("__brepWireframe", "wireframe toggle"),
    ("__brepRefSelect", "reference-selection pick {active, prompt, names, subject, path, multiple, picks, inForm}"),
    ("__brepStep", "the rolled-to feature index"),
    ("__brepParams", "inputParams of the rolled-to feature"),
    ("__brepImportPreview", "STL/OBJ import preview state (tolerances, counts, accept readiness); null when no preview is open"),
    ("__brepImportRegions", "interactive mesh import: phase, region list (proposed kind, status, residual, refits), selected region and its form; null when no preview is open"),
    ("__brepPpp", "pixels per point of the surface"),
];

/// Whether `name` is produced from app state (no layout needed).
pub fn is_state_blob(name: &str) -> bool {
    STATE_BLOBS.iter().any(|(n, _)| *n == name)
}

/// The one-line doc of a state blob.
pub fn doc(name: &str) -> &'static str {
    STATE_BLOBS.iter().find(|(n, _)| *n == name).map_or("", |(_, d)| d)
}

/// The blobs the import preview's frame publishes (it returns before the
/// rest of the shell draws).
pub const PREVIEW: &[&str] = &["__brepImportPreview", "__brepImportRegions", "__brepCamera", "__brepPpp", "__brepHistory", "__brepDocuments"];
