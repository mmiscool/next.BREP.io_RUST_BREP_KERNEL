//! [`History`] — the engine-owned, editable model recipe: the ordered feature
//! history plus the rollback index (the feature the model is currently built up
//! to). This is the SINGLE SOURCE OF TRUTH for the model — the UI keeps NO copy;
//! it mutates and reads the history only through [`crate::engine_state::EngineState`]
//! methods. That keeps one engine-owned history (UI-agnostic) and converges with
//! the in-flight "whole history in Rust" pipeline migration — later this sinks
//! into `brep-kernel-rs` proper without touching the UI.
//!
//! Rolling to a step re-runs `features[0..=rollback]` through the SAME kernel
//! pipeline: the full feature list stays in the request and `stopAtId` (the
//! editor's "stop at the expanded feature") halts execution AFTER the rolled-to
//! feature, so the kernel's incremental history cache is RETAINED across rolls
//! (no thrash) and the viewport shows the model as of that step.

use serde_json::Value;

/// The cap on the undo (and redo) stack depth — old entries fall off the bottom.
const MAX_UNDO: usize = 100;

/// A restorable model state: the whole document PLUS the rolled-to step, captured
/// together so an undo returns both the geometry and the view to the state they
/// were in right before the mutation.
#[derive(Debug, Clone)]
struct Snapshot {
    request: Value,
    rollback: usize,
    /// The parts-library block as it stood before the mutation. An `Rc`, so
    /// capturing it costs a pointer bump however large the library is — undo
    /// and redo still rewind it exactly as they did when it lived inside
    /// `request` (a redo of a component insert must restore the entry its
    /// ACOMP references).
    parts_library: std::rc::Rc<Value>,
    /// Names this undo step, so a caller can later ask whether the step it
    /// took is still the newest one ([`History::undo_top`]). Drawn from one
    /// process-wide counter; a redo entry keeps no serial of its own.
    serial: u64,
}

/// Hands out every [`Snapshot::serial`] in this process.
static NEXT_STEP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

fn next_step() -> u64 {
    NEXT_STEP.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

/// The engine-owned mutable history.
#[derive(Debug, Clone)]
pub struct History {
    /// The whole `HistoryRequest` document
    /// (`{expressions, configurator, features: [...]}`).
    ///
    /// PRIVATE and reached only through [`Self::document`] /
    /// [`Self::document_mut`] — the `_mut` door bumps [`Self::revision`], which
    /// is what lets a per-frame cache key off the document without having to
    /// enumerate the mutation sites. Naming the field `document` (not
    /// `request`) is what makes that total: the compiler, not a reviewer, finds
    /// a site that bypasses the doors.
    document: Value,
    /// A ticket drawn from one process-wide counter on construction and again on
    /// every `&mut` reach into [`Self::document`]. Deliberately coarse — it
    /// counts mutation OPPORTUNITIES, so it moves when nothing actually changed
    /// (a cache then rebuilds once, harmlessly) but it cannot fail to move when
    /// something did.
    ///
    /// Drawn from a shared counter rather than incremented per instance, because
    /// a `History` is not only mutated, it is REPLACED: `set_history_json`
    /// installs a fresh one on the engine a document ALREADY owns — the
    /// `?loadModel=` boot drain, a recovery restore, an STL import accept, New.
    /// A per-instance counter restarts there, so a replacement would hand a
    /// cache a revision it had already seen for the same document and the cache
    /// would serve the PREVIOUS model's values. One counter makes the guarantee
    /// the same for both: no two states of any `History` in this process ever
    /// share a revision.
    revision: u64,
    /// Index into `features` the model is rolled to (clamped to the last).
    rollback: usize,
    /// Undo/redo over the MODEL document. A snapshot is pushed BEFORE each model
    /// mutation (edit / add / delete / reorder); roll-to-step is view state and
    /// is NOT snapshotted. Rapid same-target edits (a slider drag) coalesce into a
    /// single undo entry via `last_edit_key`. The stacks live here in the engine
    /// core — the model is engine-owned, so its undo history is too; the UI only
    /// triggers `undo()` / `redo()`.
    undo_stack: Vec<Snapshot>,
    redo_stack: Vec<Snapshot>,
    /// The coalescing token of the most recently recorded edit (see `checkpoint`).
    last_edit_key: Option<String>,
    /// The persistent GLOBAL feature counter: bumped by one on every new-feature
    /// mint ([`Self::next_feature_id`]), so a new id is `{shortName}{counter}`.
    /// MONOTONIC and NEVER reused — deleting a feature does not free its number,
    /// and (deliberately) undo does NOT rewind it, so re-doing an add can't collide
    /// with a number already handed out. It is kept OFF `self.document` in memory
    /// (so the undo snapshots that clone `request` never rewind it) and folded into
    /// the serialized document under `"featureCounter"` so it round-trips save/load
    /// (see [`Self::request_json`] / [`Self::from_request_json`]).
    feature_counter: u64,
    /// The assemblies PARTS LIBRARY block (`partsLibrary`), kept OFF
    /// `self.document` for the same reason as `feature_counter`: every per-edit
    /// pass over the document — the undo checkpoint clone, the kernel request
    /// built by [`Self::prefix_request`], the assembly pose fold's serialize →
    /// parse round trip — would otherwise copy the whole library, which for an
    /// imported STEP assembly is megabytes of embedded part payload PER EDIT
    /// and froze the browser UI. Folded back in by [`Self::request_json`] so
    /// the SAVED document is byte-identical to before, and lifted back out by
    /// [`Self::from_request_json`] / the adopt doors.
    ///
    /// `Rc` because undo/redo MUST rewind it (a redo of a component insert has
    /// to restore the entry its ACOMP references) while a checkpoint must stay
    /// a pointer copy — the whole point of moving it off `request`.
    parts_library: std::rc::Rc<Value>,
    /// Why the latest edit of a part's pins or ports was not followed to the
    /// other side, naming the label that blocks it (see the pins-and-ports
    /// block below). `None` once an edit binds, and after undo, redo or a
    /// replaced document, which restore both sides together.
    pin_port_hold: Option<String>,
    /// A symbol edit changed the part's declared connection points, which only
    /// a history run publishes. The engine's `pump` re-runs and clears it, so
    /// no host has to remember to.
    ports_followed: bool,
    /// A ticket (from the same counter as [`Self::revision`]) redrawn on every
    /// USER edit: every undo checkpoint, coalesced or not, and every undo and
    /// redo. Machine writes (a solver fold, a parts-library heal, a
    /// `_no_undo` block) and rolling do not move it. See [`Self::edit_serial`].
    edit_serial: u64,
    /// Why this document may not change — a PLM revision that is released, or
    /// that this user has not checked out (`BREP_app`'s `Document::access`).
    /// `None`: editable, as every file-based document always is. See
    /// [`Self::lock`].
    lock: Option<String>,
    /// Where a mutation lands while [`Self::lock`] holds: a copy of the
    /// document taken by [`Self::document_mut`] and thrown away by the next.
    scratch: Value,
    /// How many mutations were refused since construction — the count a host
    /// watches to tell the user "check out to edit" ([`Self::refused_edits`]).
    refused: u64,
    /// Of those, the ones that were a USER's edit ([`Self::refused_user_edits`]).
    refused_user: u64,
}

/// Hands out every revision ticket in this process — see [`History::revision`].
/// Relaxed is enough: the value is only ever compared for equality against one
/// a single thread read earlier, never used to order anything.
static NEXT_REVISION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

fn next_revision() -> u64 {
    NEXT_REVISION.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

impl Default for History {
    fn default() -> Self {
        Self {
            document: empty_request(),
            revision: next_revision(),
            edit_serial: next_revision(),
            rollback: 0,
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            last_edit_key: None,
            feature_counter: 0,
            parts_library: std::rc::Rc::new(Value::Null),
            pin_port_hold: None,
            ports_followed: false,
            lock: None,
            scratch: Value::Null,
            refused: 0,
            refused_user: 0,
        }
    }
}

fn empty_request() -> Value {
    serde_json::json!({ "expressions": "", "configurator": {}, "features": [] })
}

/// The trailing run of ASCII digits of `id` read as a number (`"P.CU12"` → 12,
/// `"Box"` → 0). Trailing digits are ASCII (one byte each), so the slice boundary
/// is always a valid char boundary; a missing/overflowing run yields 0.
fn trailing_number(id: &str) -> u64 {
    let digit_bytes = id
        .bytes()
        .rev()
        .take_while(u8::is_ascii_digit)
        .count();
    id[id.len() - digit_bytes..].parse().unwrap_or(0)
}

impl History {
    pub(crate) fn fold_plugin_persistent_data(&mut self, id: &str, data: Value) {
        let Some(index) = self.index_of(id) else { return; };
        if self.features()[index].get("persistentData") == Some(&data) { return; }
        if let Some(features) = self.document_mut().get_mut("features").and_then(Value::as_array_mut) {
            features[index]["persistentData"] = data;
        }
    }

    /// Publish a successfully recomputed staged transaction as exactly one undo step.
    /// The caller must check its captured revision before reaching this door.
    pub(crate) fn commit_staged(&mut self, staged: &Self) -> Result<(), String> {
        if self.refuse_user() {
            return Err(format!("the document is read-only: {}", self.locked().unwrap_or("locked")));
        }
        self.checkpoint(None);
        *self.document_mut() = staged.document.clone();
        self.parts_library = staged.parts_library.clone();
        self.feature_counter = staged.feature_counter;
        self.rollback = staged.rollback;
        self.pin_port_hold = None;
        Ok(())
    }

    /// Load a whole history document (a saved part file parses as one). Rolls to
    /// the last feature. Ensures a `features` array exists.
    pub fn from_request_json(json: &str) -> Result<Self, String> {
        let mut request: Value =
            serde_json::from_str(json).map_err(|e| format!("history parse: {e}"))?;
        if !request.get("features").map(Value::is_array).unwrap_or(false) {
            if let Some(obj) = request.as_object_mut() {
                obj.insert("features".into(), Value::Array(Vec::new()));
            } else {
                request = empty_request();
            }
        }
        // Lift the persistent feature counter OUT of the document so it lives ONLY
        // in the struct field: kept off `self.document`, the undo snapshots (which
        // clone `request`) can never rewind it, and it can't be double-folded on
        // re-serialize. A document with no stored counter (fresh or saved before
        // this field existed) safe-inits below.
        let stored = request
            .as_object_mut()
            .and_then(|obj| obj.remove("featureCounter"))
            .and_then(|value| value.as_u64());
        // Lift the parts-library block out of the document for the same reason
        // (see the `parts_library` field): every per-edit copy of `request`
        // would otherwise carry megabytes of embedded part payload.
        let parts_library = request
            .as_object_mut()
            .and_then(|obj| obj.remove("partsLibrary"))
            .unwrap_or(Value::Null);
        let mut history = Self {
            document: request,
            rollback: 0,
            parts_library: std::rc::Rc::new(parts_library),
            ..Self::default()
        };
        history.rollback = history.len().saturating_sub(1);
        // Safe init when unstored: start ABOVE the largest numeric suffix already
        // present among feature ids so the next mint (`{shortName}{counter+1}`)
        // cannot collide with an existing id. This holds because no shortName ends
        // in a digit (verified — even `IMPORT3D` ends in `D`), so an id's trailing
        // digits ARE its numeric suffix and `counter+1` strictly exceeds them all.
        history.feature_counter = stored.unwrap_or_else(|| history.max_id_suffix());
        Ok(history)
    }

    /// The largest trailing-integer suffix among all existing feature ids (0 when
    /// none carry one) — the floor for a safe counter init on a document with no
    /// stored `featureCounter` (see [`Self::from_request_json`]).
    fn max_id_suffix(&self) -> u64 {
        self.features()
            .iter()
            .filter_map(|f| {
                f.get("inputParams")
                    .and_then(|p| p.get("id"))
                    .and_then(Value::as_str)
            })
            .map(trailing_number)
            .max()
            .unwrap_or(0)
    }

    /// The whole document, read-only.
    fn document(&self) -> &Value {
        &self.document
    }

    /// The whole document for mutation. Bumps [`Self::revision`] — this is the
    /// only way to reach `self.document` mutably.
    ///
    /// It is NOT the only door a cache keyed on the revision has to cover. The
    /// `partsLibrary` block is held in [`Self::parts_library`], beside the
    /// document rather than inside it, and [`Self::request_json`] folds it back
    /// in — so it is part of the saved document and has its own two setters,
    /// which bump the revision themselves. Anything else lifted out of
    /// `self.document` into a field of its own owes the same.
    ///
    /// While the history is [locked](Self::lock), the reach lands on a SCRATCH
    /// copy that the next reach replaces, and the refusal is counted: every
    /// mutation that goes through this door — a user edit, and equally a
    /// solver fold or a pin settle — leaves [`Self::request_json`] exactly as
    /// it was. The revision does not move either, since nothing changed.
    fn document_mut(&mut self) -> &mut Value {
        if self.lock.is_some() {
            self.refused += 1;
            self.scratch = self.document.clone();
            return &mut self.scratch;
        }
        self.revision = next_revision();
        &mut self.document
    }

    /// Refuse every change to the saved document until [`Self::unlock`]:
    /// [`Self::request_json`] stays byte for byte what it is now, whatever is
    /// called. `reason` is the sentence a host shows ("released", "checked out
    /// by grace", "not checked out"). Rolling ([`Self::set_rollback`]) is view
    /// state and stays free, and so does reading.
    ///
    /// The three writes that do not pass [`Self::document_mut`] — the parts
    /// library, the feature counter and the undo/redo stacks — check the lock
    /// themselves; `a_locked_history_keeps_its_saved_bytes` calls every
    /// mutator and pins the invariant.
    pub fn lock(&mut self, reason: impl Into<String>) {
        self.lock = Some(reason.into());
        // A typing run from before the lock must not coalesce with the first
        // edit after the next checkout into one undo entry.
        self.break_coalescing();
    }

    /// Allow changes again (a checkout took the lock).
    pub fn unlock(&mut self) {
        self.lock = None;
        self.scratch = Value::Null;
    }

    /// Why the document may not change, or `None` when it may.
    pub fn locked(&self) -> Option<&str> {
        self.lock.as_deref()
    }

    /// Mutations refused while locked, counted since this history was built.
    /// A host compares it frame to frame to tell the user an edit was refused.
    pub fn refused_edits(&self) -> u64 {
        self.refused
    }

    /// Refused mutations that were a user's edit: every one that takes an undo
    /// checkpoint, an undo, redo or retract, a feature id minted, a parts-library
    /// edit. A kernel write-back (a solver fold, a pin settle, a parts-library
    /// heal) is refused just the same but not counted here: it is nobody's edit,
    /// so a host tells the user about THIS count, not [`Self::refused_edits`].
    pub fn refused_user_edits(&self) -> u64 {
        self.refused_user
    }

    /// Count a refused write that does not pass [`Self::document_mut`], and
    /// say whether to refuse it.
    fn refuse(&mut self) -> bool {
        if self.lock.is_some() {
            self.refused += 1;
            return true;
        }
        false
    }

    /// [`Self::refuse`] for a write only a user makes.
    fn refuse_user(&mut self) -> bool {
        let refused = self.refuse();
        if refused {
            self.refused_user += 1;
        }
        refused
    }

    /// A monotonic counter of mutation opportunities on the document — the
    /// cache key for anything derived from it that would otherwise be rebuilt
    /// every frame (see the `__brepParams` blob in `BREP_app`).
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Moves on every user edit of the model and on every undo and redo, and
    /// on nothing else: not on a run's write-back, not on rolling. What a
    /// transient view of the model (the family-row preview) keys on to know
    /// that the user changed the model under it.
    pub fn edit_serial(&self) -> u64 {
        self.edit_serial
    }

    /// The features slice (empty if none).
    pub fn features(&self) -> &[Value] {
        self.document()
            .get("features")
            .and_then(Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    fn features_mut(&mut self) -> &mut Vec<Value> {
        let obj = self
            .document_mut()
            .as_object_mut()
            .expect("history request is a JSON object");
        obj.entry("features")
            .or_insert_with(|| Value::Array(Vec::new()));
        obj.get_mut("features")
            .and_then(Value::as_array_mut)
            .expect("features is a JSON array")
    }

    pub fn len(&self) -> usize {
        self.features().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The rolled-to index, clamped to a valid feature (0 when empty).
    pub fn rollback(&self) -> usize {
        self.rollback.min(self.len().saturating_sub(1))
    }

    pub fn set_rollback(&mut self, index: usize) {
        self.rollback = if self.is_empty() {
            0
        } else {
            index.min(self.len() - 1)
        };
        // Rolling to a step is a view move, not a model edit: it records NO undo
        // snapshot, but it DOES break the edit-coalescing run so the next edit
        // starts a fresh undo entry rather than merging with a pre-roll edit.
        self.last_edit_key = None;
    }

    pub fn feature_type(&self, index: usize) -> Option<String> {
        self.features()
            .get(index)?
            .get("type")
            .and_then(Value::as_str)
            .map(String::from)
    }

    pub fn feature_id(&self, index: usize) -> Option<String> {
        self.features()
            .get(index)?
            .get("inputParams")
            .and_then(|p| p.get("id"))
            .and_then(Value::as_str)
            .map(String::from)
    }

    pub fn index_of(&self, id: &str) -> Option<usize> {
        self.features().iter().position(|f| {
            f.get("inputParams")
                .and_then(|p| p.get("id"))
                .and_then(Value::as_str)
                == Some(id)
        })
    }

    /// The `inputParams` document of the feature at `index` (for the dialog).
    pub fn feature_params(&self, index: usize) -> Option<Value> {
        self.features().get(index)?.get("inputParams").cloned()
    }

    /// SOLVER write-back fold (the assembly pose-authority contract): set ONE
    /// `inputParams` key of the feature whose `inputParams.id == id`, WITHOUT an
    /// undo checkpoint and WITHOUT breaking edit coalescing — a solve write-back
    /// is the kernel adopting its own result, not a user edit, so it must never
    /// mint an undo entry (undoing a user action then re-running re-solves and
    /// re-folds anyway). Returns whether a feature matched.
    pub fn fold_param_no_undo(&mut self, id: &str, key: &str, value: Value) -> bool {
        let Some(index) = self.index_of(id) else {
            return false;
        };
        if let Some(feature) = self.features_mut().get_mut(index) {
            if let Some(params) = feature
                .get_mut("inputParams")
                .and_then(Value::as_object_mut)
            {
                params.insert(key.to_string(), value);
                return true;
            }
        }
        false
    }

    pub fn set_feature_params(&mut self, index: usize, params: Value) {
        if index >= self.len() {
            return;
        }
        // Coalesce a slider drag (many consecutive edits of the SAME feature) into
        // one undo entry, keyed by the feature index.
        self.checkpoint(Some(&format!("param:{index}")));
        if let Some(feat) = self.features_mut().get_mut(index) {
            if let Some(obj) = feat.as_object_mut() {
                obj.insert("inputParams".into(), params);
            }
        }
    }

    /// Replace the `inputParams` of MANY features as ONE mutation —
    /// [`Self::set_feature_params`]'s batch sibling, and the reason it exists:
    /// a packed BOM row rolls up N occurrences, so editing one cell writes N
    /// features. A `set_feature_params` loop would mint N undo entries (each
    /// keyed `param:{index}`, so none of them coalesce with each other), and
    /// the user would have to press undo N times to take back ONE edit.
    ///
    /// Exactly ONE checkpoint covers the whole batch. The coalesce key names
    /// the SET of indices, so a run of edits to the SAME set (typing into one
    /// packed cell) still merges into a single entry, while switching to a
    /// different set starts a new one — the `param:{index}` rule, lifted to a
    /// group. Out-of-range indices are skipped; an empty batch is a no-op (no
    /// checkpoint, so a fan-out that matched nothing leaves no empty entry).
    pub fn set_many_feature_params(&mut self, edits: &[(usize, Value)]) {
        let len = self.len();
        let mut in_range: Vec<&(usize, Value)> =
            edits.iter().filter(|(index, _)| *index < len).collect();
        if in_range.is_empty() {
            return;
        }
        // The key must not depend on the caller's ordering, or the same packed
        // row could produce two different keys and stop coalescing.
        in_range.sort_by_key(|(index, _)| *index);
        let key: Vec<String> = in_range
            .iter()
            .map(|(index, _)| index.to_string())
            .collect();
        self.checkpoint(Some(&format!("params:{}", key.join(","))));
        for (index, params) in in_range {
            if let Some(feature) = self.features_mut().get_mut(*index) {
                if let Some(object) = feature.as_object_mut() {
                    object.insert("inputParams".into(), params.clone());
                }
            }
        }
    }

    pub fn push_feature(&mut self, feature: Value) {
        self.checkpoint(None);
        self.features_mut().push(feature);
    }

    /// Append MANY features as ONE mutation — the batch an import lane needs
    /// (`EngineState::add_features`). Distinct from a `push_feature` loop in the
    /// two ways that matter: exactly ONE undo checkpoint (an N-instance assembly
    /// import undoes in one step, not N), and the caller re-runs once instead of
    /// once per feature. No-op for an empty batch (no checkpoint, so an import
    /// that produced nothing leaves no empty undo entry).
    pub fn push_features(&mut self, features: Vec<Value>) {
        if features.is_empty() {
            return;
        }
        self.checkpoint(None);
        self.features_mut().extend(features);
    }

    pub fn remove_feature(&mut self, index: usize) {
        if index >= self.len() {
            return;
        }
        self.checkpoint(None);
        self.features_mut().remove(index);
    }

    pub fn swap(&mut self, a: usize, b: usize) {
        let len = self.len();
        if a < len && b < len && a != b {
            self.checkpoint(None);
            self.features_mut().swap(a, b);
        }
    }

    /// Mint the id for a NEW feature: `{base}{N}` where `base` is the feature's
    /// shortName and `N` is this history's persistent GLOBAL counter, bumped by one
    /// on every mint (`P.CU` → `P.CU7`, `S` → `S8`). GLOBAL across all feature
    /// types, MONOTONIC, and NEVER reused — a delete does not free a number and
    /// undo does not rewind the counter — and it persists across save/load, so two
    /// features can never receive the same id over the document's whole lifetime.
    pub fn next_feature_id(&mut self, base: &str) -> String {
        // Locked, the counter is saved state and does not move; the id is
        // still minted, for a feature whose push will be refused anyway.
        if self.refuse_user() {
            return format!("{base}{}", self.feature_counter + 1);
        }
        self.feature_counter += 1;
        format!("{base}{}", self.feature_counter)
    }

    /// Commit IDs reserved on a transaction's cloned history. Document adoption
    /// deliberately ignores serialized counters, so the transaction must carry
    /// its allocation high-water mark separately. Explicit committed IDs also
    /// set a floor; undo keeps this monotonic, while rollback restores a clone.
    pub(crate) fn retain_feature_allocations(&mut self, staged: &Self) {
        if self.refuse_user() {
            return;
        }
        self.feature_counter = self
            .feature_counter
            .max(staged.feature_counter)
            .max(self.max_id_suffix());
    }

    /// The `stopAtId`-truncated request that stops AFTER the rolled-to feature —
    /// the roll-to-step request the pipeline runs. Empty history → empty request.
    pub fn prefix_request(&self) -> Value {
        let mut request = self.document().clone();
        if let Some(id) = self.feature_id(self.rollback()) {
            if let Some(obj) = request.as_object_mut() {
                obj.insert("stopAtId".into(), Value::String(id));
            }
        }
        request
    }

    /// The request that stops BEFORE feature `index` — the scene that feature
    /// resolves its references against, wherever the model is rolled to. `None`
    /// when there is no feature at `index`.
    pub fn request_before(&self, index: usize) -> Option<Value> {
        let id = self.feature_id(index)?;
        let mut request = self.document().clone();
        request.as_object_mut()?.insert("stopBeforeId".into(), Value::String(id));
        Some(request)
    }

    /// The tree listing for the UI: `{ step, features: [{index, type, id}] }`.
    pub fn listing_json(&self) -> String {
        let features: Vec<Value> = self
            .features()
            .iter()
            .enumerate()
            .map(|(index, _)| {
                serde_json::json!({
                    "index": index,
                    "type": self.feature_type(index).unwrap_or_else(|| "?".into()),
                    "id": self.feature_id(index).unwrap_or_else(|| "(no id)".into()),
                })
            })
            .collect();
        serde_json::json!({ "step": self.rollback(), "features": features }).to_string()
    }

    /// The whole request document (for persistence / debugging), with the
    /// persistent global feature counter folded back in under `"featureCounter"`
    /// so it round-trips through save/load (the twin of [`Self::from_request_json`],
    /// which lifts it back out). Written only when non-zero, so a document that has
    /// never minted a feature persists byte-for-byte as before (mirrors the
    /// metadata field's "un-annotated model persists unchanged" convention).
    pub fn request_json(&self) -> String {
        let has_library = self
            .parts_library
            .as_object()
            .map(|map| !map.is_empty())
            .unwrap_or(false);
        if self.feature_counter == 0 && !has_library {
            return self.document().to_string();
        }
        let mut document = self.document().clone();
        if let Some(obj) = document.as_object_mut() {
            if self.feature_counter != 0 {
                obj.insert("featureCounter".into(), Value::from(self.feature_counter));
            }
            if has_library {
                obj.insert("partsLibrary".into(), (*self.parts_library).clone());
            }
        }
        document.to_string()
    }

    /// The document WITHOUT the parts-library block — for the kernel round
    /// trips that never read it. The assembly pose fold
    /// (`assembly_apply_document_json`) only rewrites the `assembly` block and
    /// per-feature `inputParams`, so handing it the library would serialize,
    /// parse and re-serialize megabytes of part payload on every edit for
    /// nothing. [`Self::request_json`] is the SAVE door and still carries it.
    pub fn request_json_without_parts_library(&self) -> String {
        if self.feature_counter == 0 {
            return self.document().to_string();
        }
        let mut document = self.document().clone();
        if let Some(obj) = document.as_object_mut() {
            obj.insert("featureCounter".into(), Value::from(self.feature_counter));
        }
        document.to_string()
    }

    // --- Undo / redo over the model document ------------------------------

    fn snapshot(&self) -> Snapshot {
        Snapshot {
            request: self.document().clone(),
            rollback: self.rollback,
            parts_library: self.parts_library.clone(),
            serial: next_step(),
        }
    }

    fn restore(&mut self, snap: Snapshot) {
        self.edit_serial = next_revision();
        *self.document_mut() = snap.request;
        self.parts_library = snap.parts_library;
        // A snapshot holds both sides as they stood together, so whatever the
        // last pin or port edit held is no longer on screen.
        self.pin_port_hold = None;
        let last = self.len().saturating_sub(1);
        self.rollback = snap.rollback.min(last);
    }

    /// Record a pre-mutation snapshot for undo. `coalesce_key` groups a run of
    /// rapid same-target edits (one slider drag) into a SINGLE undo entry: while
    /// the same non-empty key repeats, no new snapshot is pushed. A `None` key
    /// never coalesces, so every structural add/delete/reorder is its own entry.
    /// Any new snapshot clears the redo stack (a fresh edit forks the timeline)
    /// and the oldest entry falls off once the stack passes [`MAX_UNDO`].
    ///
    /// A run has no time limit: only another key, an undo or
    /// [`Self::break_coalescing`] ends it. The PMI label drag and the sheet
    /// view, dimension and ordinate moves call that on release; nothing else
    /// does, and a typing run has no release. So a param slider or
    /// field (`param:{index}`) or an eCAD field (`component:{id}:text`) edited,
    /// left, and edited again later with nothing in between is ONE undo step
    /// across the pause, even across a Save. An eCAD host should call it when
    /// focus leaves the editor's fields. Param runs are left as they are, a
    /// decision not yet taken.
    fn checkpoint(&mut self, coalesce_key: Option<&str>) {
        // Locked, the edit this checkpoint precedes is refused at
        // `document_mut`: no undo entry for an edit that never happened.
        if self.lock.is_some() {
            self.refused_user += 1;
            return;
        }
        // Before the coalesce return: the second keystroke of a typing run is
        // an edit too.
        self.edit_serial = next_revision();
        if coalesce_key.is_some() && coalesce_key == self.last_edit_key.as_deref() {
            return;
        }
        self.undo_stack.push(self.snapshot());
        if self.undo_stack.len() > MAX_UNDO {
            self.undo_stack.remove(0);
        }
        self.redo_stack.clear();
        self.last_edit_key = coalesce_key.map(str::to_string);
    }

    /// Whether an undo step is available (for enabling the toolbar button).
    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    /// Whether a redo step is available.
    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Undo the last model mutation: push the current state onto the redo stack
    /// and restore the previous document + rolled-to step. Returns whether it
    /// changed anything (false when the undo stack is empty).
    pub fn undo(&mut self) -> bool {
        if self.refuse_user() {
            return false;
        }
        let Some(prev) = self.undo_stack.pop() else {
            return false;
        };
        self.redo_stack.push(self.snapshot());
        self.restore(prev);
        // A distinct undo breaks any coalescing run so the next edit is fresh.
        self.last_edit_key = None;
        true
    }

    /// Redo the last undone model mutation (symmetric with [`Self::undo`]).
    pub fn redo(&mut self) -> bool {
        if self.refuse_user() {
            return false;
        }
        let Some(next) = self.redo_stack.pop() else {
            return false;
        };
        self.undo_stack.push(self.snapshot());
        self.restore(next);
        self.last_edit_key = None;
        true
    }
}

// ============================================================================
// Assembly document accessors (appended — the assemblies Wave-3 slice). A
// SEPARATE `impl` block so concurrent edits to the primary block don't
// conflict; purely additive over the existing history API.
// ============================================================================
impl History {
    /// ADOPT a whole replacement document — the assembly FOLD's write-back lane
    /// (`assembly_apply_document_json` returns the document with the solved
    /// `assembly` block + poses/isFixed folded into the features; the engine
    /// adopts it before persisting or re-running — the pose-authority contract).
    ///
    /// `checkpoint` chooses the undo semantics: a USER constraint mutation
    /// records an undo snapshot (so constraint edits stay undoable like any
    /// model edit); the silent post-run pose fold passes `false` (solver
    /// write-back is not a user edit — undoing the user's LAST edit must not
    /// strand an extra fold step in between).
    ///
    /// A stray `featureCounter` in the adopted document is stripped (the
    /// in-memory counter stays authoritative — `request_json` re-folds it on
    /// serialize, exactly like `from_request_json` lifts it on load). The
    /// rollback index is preserved (the fold never changes the feature count).
    pub fn adopt_document(&mut self, document_json: &str) -> Result<(), String> {
        // Locked, say so: a caller that checks the result (an eCAD block sync
        // recording what it wrote) must not believe a refused write landed.
        if let Some(reason) = &self.lock {
            let error = format!("the document is read-only: {reason}");
            self.refused += 1;
            return Err(error);
        }
        let mut document: Value = serde_json::from_str(document_json)
            .map_err(|error| format!("adopt document parse: {error}"))?;
        if !document.is_object() {
            return Err("adopt document: not a JSON object".into());
        }
        let mut adopted_library = None;
        if let Some(obj) = document.as_object_mut() {
            obj.remove("featureCounter");
            // A block PRESENT in the adopted document replaces the field; one
            // ABSENT leaves it alone (the fold lane is handed a library-free
            // document by `request_json_without_parts_library` and must not
            // silently drop the library on the way back).
            adopted_library = obj.remove("partsLibrary");
            if !obj.get("features").map(Value::is_array).unwrap_or(false) {
                obj.insert("features".into(), Value::Array(Vec::new()));
            }
        }
        *self.document_mut() = document;
        // A replaced document is not the one a pin or port edit was held on.
        self.pin_port_hold = None;
        if let Some(library) = adopted_library {
            self.set_parts_library(library);
            }
        Ok(())
    }

    /// Same adoption with an undo snapshot recorded FIRST (never coalesced) —
    /// the user-mutation twin of [`Self::adopt_document`].
    pub fn adopt_document_checkpointed(&mut self, document_json: &str) -> Result<(), String> {
        // Validate BEFORE snapshotting so a parse failure never pushes a
        // phantom undo entry.
        let probe: Value = serde_json::from_str(document_json)
            .map_err(|error| format!("adopt document parse: {error}"))?;
        if !probe.is_object() {
            return Err("adopt document: not a JSON object".into());
        }
        self.checkpoint(None);
        self.adopt_document(document_json)
    }

    /// Replace the document's `partsLibrary` block (the assemblies parts
    /// library) — the caller feeds `brep_kernel::parts_library_json()` here
    /// (never an echo of a loaded block), so SAVE serializes the kernel
    /// store with its heals and GC. An empty map clears the field, so a
    /// non-assembly document serializes byte-identically to before.
    ///
    /// The block is held in the `parts_library` FIELD rather than on
    /// `self.document`; [`Self::request_json`] folds it back into the saved
    /// document. The on-disk shape is unchanged.
    pub fn set_parts_library(&mut self, library: Value) {
        if self.refuse() {
            return;
        }
        let empty = library.as_object().map(|m| m.is_empty()).unwrap_or(true);
        self.parts_library = std::rc::Rc::new(if empty { Value::Null } else { library });
        // The block is part of the saved document, so this is a document
        // mutation and owes the revision a tick — `document_mut` never sees it.
        self.revision = next_revision();
    }

    /// Replace the `partsLibrary` block as a USER EDIT: one undo checkpoint,
    /// and the mirror flag CLEARED so the next run ships the block to the
    /// runner instead of assuming the kernel store already agrees.
    ///
    /// [`Self::set_parts_library`] is the other door and means the opposite —
    /// "this block came OUT of the kernel store" — so it must not be reused
    /// here: a part-attribute edit is authored on this side and the runner has
    /// never seen it. `coalesce_key` follows the `param:{index}` rule so a run
    /// of keystrokes into one attribute is one undo entry.
    pub fn set_parts_library_edited(&mut self, library: Value, coalesce_key: Option<&str>) {
        if self.refuse_user() {
            return;
        }
        self.checkpoint(coalesce_key);
        let empty = library.as_object().map(|m| m.is_empty()).unwrap_or(true);
        self.parts_library = std::rc::Rc::new(if empty { Value::Null } else { library });
        // As [`Self::set_parts_library`]: `checkpoint` records an undo entry
        // and moves nothing a cache can see.
        self.revision = next_revision();
    }

    /// The `partsLibrary` block (`Value::Null` when the document has none).
    pub fn parts_library(&self) -> &Value {
        &self.parts_library
    }

    /// The document's `assembly` block (`{constraints, idCounter}`), if any.
    pub fn assembly_block(&self) -> Option<&Value> {
        self.document().get("assembly")
    }

    /// Replace the `assembly` block (`{constraints, idCounter}`) — the STATE
    /// half of the assembly write-back, which the engine folds from the run's
    /// reply after every run and from the live session after a constraint
    /// mutation. `checkpoint` = true makes it an undoable USER edit (a
    /// mutation); false writes it silently, because a solver adopting its own
    /// result is not a user edit and must never mint an undo entry — undoing a
    /// user action then re-running re-solves and re-folds anyway.
    pub fn set_assembly_block(&mut self, block: Value, checkpoint: bool) {
        if checkpoint {
            self.checkpoint(None);
        }
        if let Some(object) = self.document_mut().as_object_mut() {
            object.insert("assembly".into(), block);
        }
    }

    /// The document's `wireHarness` block (`{connections, idCounter,
    /// buildBundles}`), if any.
    pub fn wire_harness_block(&self) -> Option<&Value> {
        self.document().get("wireHarness")
    }

    /// Replace (or, with `None`, remove) the `wireHarness` block. A USER edit:
    /// snapshotted for undo, never coalesced — each add / edit / remove of a
    /// connection is its own undo step, like a feature add or delete.
    pub fn set_wire_harness_block(&mut self, block: Option<Value>) {
        self.checkpoint(None);
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert("wireHarness".into(), block);
                }
                None => {
                    object.remove("wireHarness");
                }
            }
        }
    }
}

// ============================================================================
// PMI block accessors (the PMI workbench slice). A SEPARATE `impl` block —
// purely additive over the history API.
// ============================================================================
impl History {
    /// The document's `pmi` block (`{views, idCounter}`), if any.
    pub fn pmi_block(&self) -> Option<&Value> {
        self.document().get("pmi")
    }

    /// Replace (or, with `None`, remove) the `pmi` block as a USER edit:
    /// snapshotted for undo. `coalesce_key` groups a run of rapid same-target
    /// edits (a label drag: `pmi:label:{id}`) into ONE undo entry; `None`
    /// makes the edit its own step (a view capture, an annotation add / edit
    /// / delete).
    pub fn set_pmi_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.checkpoint(coalesce_key);
        self.put_pmi_block(block);
    }

    /// Replace the `pmi` block WITHOUT an undo checkpoint — for a write that
    /// belongs to the checkpoint just taken (an import lifting a file's PMI
    /// beside the feature it added, so one undo removes both).
    pub fn set_pmi_block_no_undo(&mut self, block: Option<Value>) {
        self.put_pmi_block(block);
    }

    fn put_pmi_block(&mut self, block: Option<Value>) {
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert("pmi".into(), block);
                }
                None => {
                    object.remove("pmi");
                }
            }
        }
    }

    /// End a coalescing run (a label drag released): the next edit with the
    /// same key starts a fresh undo entry.
    pub fn break_coalescing(&mut self) {
        self.last_edit_key = None;
    }
}

// ============================================================================
// SHEETS block accessors (the drawing-sheet slice).
//
// `sheets` is a top-level document key like `pmi` and `partAttributes`: a
// sheet places saved PMI views and adds no geometry, so the kernel never needs
// it and `HistoryRequest` has no field for it — the document's unknown-key
// pass-through ([`Self::request_json`] serializes the raw document) is what
// round-trips it through save, load and undo. Absent on every document without
// a sheet, so those files re-serialize byte-identically.
// ============================================================================
impl History {
    /// The document's `sheets` block (`{sheets, idCounter}`), if any.
    pub fn sheets_block(&self) -> Option<&Value> {
        self.document().get(crate::sheets::SHEETS)
    }

    /// Replace (or, with `None`, remove) the `sheets` block as a USER edit:
    /// snapshotted for undo. `coalesce_key` groups a run of rapid same-target
    /// edits (dragging a placed view: `sheet:view:{id}`) into ONE undo entry,
    /// exactly as the PMI label drag's does.
    pub fn set_sheets_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.checkpoint(coalesce_key);
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert(crate::sheets::SHEETS.into(), block);
                }
                None => {
                    object.remove(crate::sheets::SHEETS);
                }
            }
        }
    }
}

// ============================================================================
// PART ATTRIBUTES block accessors — the BOM attributes of the document ITSELF.
//
// `partAttributes` is a top-level document key (see
// `engine_state::bom`'s attributes block), so a document carries the BOM data
// of the PART it is. On a part in an assembly's library that record is written
// through the library entry's embedded document; on the document you have OPEN
// it is written here, by the toolbar's Properties dialog. Same key, same
// shape, two doors — which is what makes a part's Part Number the same value
// whether it is read from the assembly's BOM or from the part's own tab.
// ============================================================================
impl History {
    /// The document's own `partAttributes` record, if any.
    pub fn part_attributes_block(&self) -> Option<&Value> {
        self.document().get(crate::engine_state::PART_ATTRIBUTES)
    }

    /// Replace (or, with `None`, remove) the document's own `partAttributes`
    /// record as a USER edit: snapshotted for undo. `coalesce_key` groups a run
    /// of edits to ONE field (a typing run in the Properties dialog) into a
    /// single undo entry, exactly as the PMI block's does.
    pub fn set_part_attributes_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.checkpoint(coalesce_key);
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert(crate::engine_state::PART_ATTRIBUTES.into(), block);
                }
                None => {
                    object.remove(crate::engine_state::PART_ATTRIBUTES);
                }
            }
        }
    }
}

// ============================================================================
// Expressions / configurator accessors (appended — the expressions/parameters
// panel slice). A SEPARATE `impl` block so concurrent edits to the primary block
// don't conflict; purely additive over the existing history API.
//
// The history document carries an `expressions` source string — the variable
// sheet feature params evaluate against (a numeric param may be the string
// `"boxW"`, evaluated by the pipeline's shared expression env). The panel edits
// this and re-runs; the `configurator` object (typed named inputs) is exposed
// read-only for display.
// ============================================================================
impl History {
    /// The history document's `expressions` source string (empty when absent or
    /// stored as `null`). The panel's editor binds to this.
    pub fn expressions(&self) -> String {
        self.document()
            .get("expressions")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string()
    }

    /// Replace the `expressions` source string. Snapshotted for undo, coalescing a
    /// run of keystroke edits into ONE undo entry (like a slider drag) via the
    /// shared `"expressions"` coalesce key, so a distinct add/edit/roll starts a
    /// fresh entry. A no-op re-set (same text) still records under the same key.
    pub fn set_expressions(&mut self, expressions: &str) {
        self.checkpoint(Some("expressions"));
        if let Some(obj) = self.document_mut().as_object_mut() {
            obj.insert(
                "expressions".into(),
                Value::String(expressions.to_string()),
            );
        }
    }

    /// The `configurator` object (typed named inputs), or `{}` when absent —
    /// read-only for the panel's display (deeper configurator editing deferred).
    pub fn configurator(&self) -> Value {
        self.document()
            .get("configurator")
            .cloned()
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()))
    }
}

// ============================================================================
// Feature `persistentData` accessors (appended — the engine-native sketch mode
// slice). A SEPARATE `impl` block (like the expressions accessors above) so
// concurrent edits don't conflict; purely additive over the primary history API.
//
// A feature's `persistentData` is the kernel-persisted, non-input state (a
// SKETCH feature stores its solved `{points, geometries, constraints}` under the
// `sketch` key and the plane `basis` there). Sketch mode reads that state on
// enter and writes the edited doc back on commit — mirroring how the ref-select
// slice reads/writes `inputParams` through `feature_params` / `set_feature_params`.
// ============================================================================
impl History {
    /// The `persistentData` document of the feature at `index` (`None` if the
    /// feature or the field is absent) — the read twin of [`Self::feature_params`].
    pub fn feature_persistent_data(&self, index: usize) -> Option<Value> {
        self.features().get(index)?.get("persistentData").cloned()
    }

    /// Set a single `key` inside the feature-at-`index`'s `persistentData` object,
    /// creating (or replacing a non-object) `persistentData` as needed. Snapshotted
    /// for undo (a structural edit — never coalesced), like an add/delete.
    pub fn set_feature_persistent_field(&mut self, index: usize, key: &str, value: Value) {
        self.set_feature_persistent_field_coalesced(index, key, value, None);
    }

    /// [`Self::set_feature_persistent_field`] with an optional COALESCE key: a
    /// run of same-key writes (a gizmo drag moving a spline anchor, frame after
    /// frame) records ONE undo entry, exactly as a slider drag on a param does.
    pub fn set_feature_persistent_field_coalesced(
        &mut self,
        index: usize,
        key: &str,
        value: Value,
        coalesce_key: Option<&str>,
    ) {
        if index >= self.len() {
            return;
        }
        self.checkpoint(coalesce_key);
        if let Some(feat) = self.features_mut().get_mut(index).and_then(Value::as_object_mut) {
            let entry = feat
                .entry("persistentData")
                .or_insert_with(|| Value::Object(serde_json::Map::new()));
            if !entry.is_object() {
                *entry = Value::Object(serde_json::Map::new());
            }
            if let Some(obj) = entry.as_object_mut() {
                obj.insert(key.to_string(), value);
            }
        }
    }

    /// Write several `persistentData` fields of the feature at `index` as ONE
    /// user edit: one undo checkpoint covers them all, and there is none when
    /// every field already holds its value, so an action that changed nothing
    /// leaves no undo step that does nothing. Returns whether anything changed.
    ///
    /// Leaving a sketch writes its geometry, its dimension label offsets and
    /// its external references this way. Written one field at a time they were
    /// three undo steps, and only the third Ctrl+Z took the drawing back.
    pub fn set_feature_persistent_fields(&mut self, index: usize, fields: Vec<(&str, Value)>) -> bool {
        let Some(feature) = self.features().get(index) else {
            return false;
        };
        let current = feature.get("persistentData");
        if fields.iter().all(|(key, value)| current.and_then(|data| data.get(*key)) == Some(value)) {
            return false;
        }
        self.checkpoint(None);
        if let Some(feat) = self.features_mut().get_mut(index).and_then(Value::as_object_mut) {
            let entry = feat
                .entry("persistentData")
                .or_insert_with(|| Value::Object(serde_json::Map::new()));
            if !entry.is_object() {
                *entry = Value::Object(serde_json::Map::new());
            }
            if let Some(obj) = entry.as_object_mut() {
                for (key, value) in fields {
                    obj.insert(key.to_string(), value);
                }
            }
        }
        true
    }

    /// The undo step on top of the stack right now, as a token for
    /// [`Self::retract`] and [`Self::set_feature_persistent_fields_in_step`];
    /// `None` when there is nothing to undo.
    pub fn undo_top(&self) -> Option<u64> {
        self.undo_stack.last().map(|snapshot| snapshot.serial)
    }

    /// Take back undo step `step` as though its edit had never been made:
    /// restore the state it holds and DROP it, leaving no redo entry. Only while
    /// `step` is still the newest step; otherwise nothing happens and this
    /// returns false, and the caller undoes the ordinary way.
    ///
    /// Cancelling a sketch created in the same session uses it, so the cancel
    /// leaves no undo step behind: two presses, one of which brought the
    /// cancelled sketch back, before. It cannot restore a redo stack the step's
    /// own edit cleared when it was taken. Neither could the cancel it replaces.
    pub fn retract(&mut self, step: u64) -> bool {
        if self.refuse_user() {
            return false;
        }
        if self.undo_top() != Some(step) {
            return false;
        }
        let Some(snapshot) = self.undo_stack.pop() else {
            return false;
        };
        self.restore(snapshot);
        self.last_edit_key = None;
        true
    }

    /// [`Self::set_feature_persistent_fields`] as part of undo step `step`,
    /// with no checkpoint of its own, while `step` is still the newest step;
    /// otherwise an ordinary write with its own step. Finishing a sketch
    /// created in the same session writes this way, so creating, drawing and
    /// finishing it is one undo step.
    pub fn set_feature_persistent_fields_in_step(
        &mut self,
        index: usize,
        fields: Vec<(&str, Value)>,
        step: u64,
    ) -> bool {
        if self.undo_top() != Some(step) {
            return self.set_feature_persistent_fields(index, fields);
        }
        let Some(feat) = self.features_mut().get_mut(index).and_then(Value::as_object_mut) else {
            return false;
        };
        let entry = feat
            .entry("persistentData")
            .or_insert_with(|| Value::Object(serde_json::Map::new()));
        if !entry.is_object() {
            *entry = Value::Object(serde_json::Map::new());
        }
        if let Some(obj) = entry.as_object_mut() {
            for (key, value) in fields {
                obj.insert(key.to_string(), value);
            }
        }
        true
    }
}

// ============================================================================
// eCAD block accessors (the eCAD documents slice).
//
// The four eCAD workbenches keep their documents as top-level keys: `diagram`
// (the wiring diagram) and `pcb` (schematic and board) are blocks of an
// ASSEMBLY document, since a board is an assembly; `symbol` and `pads` are
// blocks of a PART document, since a part carries its own symbol and pads.
// Like `sheets`, the kernel never reads them and `HistoryRequest` has no field
// for them: the raw document is what carries them through save, load, undo and
// the assembly fold. Absent on every document the eCAD editors never wrote, so
// those files re-serialize byte-identically.
//
// None of these setters runs the history. The tab's dirty dot and the autosave
// see the edit through [`History::revision`], which every write here moves. The
// two symbol setters also carry a pin edit to the part's PORT features (the
// pins-and-ports block below), and the engine's `pump` runs the history when
// that moved a port.
// ============================================================================

/// The assembly document's wiring diagram (an eCAD `Document`).
pub const DIAGRAM: &str = "diagram";
/// The assembly document's schematic and board (an eCAD `Document`).
pub const PCB: &str = "pcb";
/// The part document's schematic symbol (an eCAD `Symbol`). Defined AS the
/// kernel's key: the pin/port binding (`brep_kernel` `part_pins`) reads the same
/// block, so the two can never name different keys.
pub const SYMBOL: &str = brep_kernel::SYMBOL_BLOCK;
/// The part document's footprint pads (an eCAD `Footprint`), defined AS the
/// kernel's key for the same reason.
pub const PADS: &str = brep_kernel::PADS_BLOCK;
/// The part's declared connection points (`ports.rs`) — data, not history,
/// and a sibling of `symbol` and `pads`.
pub const PORTS: &str = brep_kernel::PORTS_BLOCK;

impl History {
    /// The document's `diagram` block, if any.
    pub fn diagram_block(&self) -> Option<&Value> {
        self.document().get(DIAGRAM)
    }

    /// The document's `pcb` block, if any.
    pub fn pcb_block(&self) -> Option<&Value> {
        self.document().get(PCB)
    }

    /// The document's `symbol` block, if any.
    pub fn symbol_block(&self) -> Option<&Value> {
        self.document().get(SYMBOL)
    }

    /// The document's `pads` block, if any.
    pub fn pads_block(&self) -> Option<&Value> {
        self.document().get(PADS)
    }

    /// The document's `ports` block (its declared connection points), if any.
    pub fn ports_block(&self) -> Option<&Value> {
        self.document().get(PORTS)
    }

    /// The part's declared connection points, in block order.
    pub fn declared_points(&self) -> Vec<brep_kernel::DeclaredPoint> {
        brep_kernel::declared_points(self.document())
    }

    /// The symbol's pin labels in pin order, or `None` without a symbol.
    pub fn pin_labels(&self) -> Option<Vec<String>> {
        brep_kernel::pin_labels(self.document())
    }

    /// The symbol's pins with the UNIT each belongs to, in pin order, or `None`
    /// without a symbol. [`Self::pin_labels`] with the unit kept, for a caller
    /// that must say WHICH group a pin binds into.
    pub fn pins(&self) -> Option<Vec<brep_kernel::DeclaredPin>> {
        brep_kernel::pins(self.document())
    }

    /// The port group the pins of symbol unit `unit` bind into, when one is
    /// mapped. The unit map's rule is the kernel's ([`brep_kernel::unit_groups`]).
    pub fn port_group_for_unit(&self, unit: u32) -> Option<String> {
        brep_kernel::port_group_for_unit(self.document(), unit)
    }

    /// Replace (or, with `None`, remove) the `diagram` block as a USER edit.
    /// See [`Self::set_ecad_block`] for `coalesce_key`.
    pub fn set_diagram_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.set_ecad_block(DIAGRAM, block, coalesce_key);
    }

    /// Replace (or, with `None`, remove) the `pcb` block as a USER edit.
    /// See [`Self::set_ecad_block`] for `coalesce_key`.
    pub fn set_pcb_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.set_ecad_block(PCB, block, coalesce_key);
    }

    /// Replace (or, with `None`, remove) the `symbol` block as a USER edit,
    /// and follow it to the part's ports in the same undo step (see
    /// [`Self::follow_pins`]). See [`Self::set_ecad_block`] for `coalesce_key`.
    pub fn set_symbol_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.set_ecad_block(SYMBOL, block, coalesce_key);
        self.follow_pins(None);
    }

    /// Replace (or, with `None`, remove) the `pads` block as a USER edit.
    /// See [`Self::set_ecad_block`] for `coalesce_key`.
    pub fn set_pads_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.set_ecad_block(PADS, block, coalesce_key);
    }

    /// Write the `symbol` block WITHOUT an undo checkpoint, for a write that
    /// belongs to the checkpoint just taken: a KiCad import adds its Import 3D
    /// feature (one checkpoint) and writes the symbol and pads it read beside
    /// it, so one undo removes the whole import. The PMI import's
    /// [`Self::set_pmi_block_no_undo`] is the precedent.
    ///
    /// Skipping the checkpoint is deliberate, and costs the tab nothing: the
    /// write still goes through `document_mut`, which moves the revision the
    /// unsaved dot and the autosave key on.
    pub fn set_symbol_block_no_undo(&mut self, block: Option<Value>) {
        // No checkpoint marks where this write starts, so its base is the
        // document just before it.
        let base = self.document().clone();
        self.put_ecad_block(SYMBOL, block);
        self.follow_pins(Some(base));
    }

    /// The `pads` twin of [`Self::set_symbol_block_no_undo`].
    pub fn set_pads_block_no_undo(&mut self, block: Option<Value>) {
        self.put_ecad_block(PADS, block);
    }

    /// Replace (or, with `None`, remove) the `ports` block as a USER edit, and
    /// follow it to the part's pins in the same undo step (see
    /// [`Self::follow_ports`]).
    pub fn set_ports_block(&mut self, block: Option<Value>, coalesce_key: Option<&str>) {
        self.set_ecad_block(PORTS, block, coalesce_key);
        self.follow_ports();
        self.ports_followed = true;
    }

    /// The `ports` twin of [`Self::set_symbol_block_no_undo`] — for a write
    /// that belongs to the checkpoint just taken (the KiCad import).
    pub fn set_ports_block_no_undo(&mut self, block: Option<Value>) {
        self.put_ecad_block(PORTS, block);
        self.ports_followed = true;
    }

    /// Write block `key`, snapshotted for undo. `coalesce_key` is eCAD's own
    /// `Change::coalesce`, passed as the editor reported it: typing in one
    /// field of one object reports `component:{uuid}:text` on every frame, and
    /// a run of writes with that key is ONE undo entry. It is namespaced by the
    /// block here (`diagram:component:{uuid}:text`), so a run in one block never
    /// swallows an edit to another. `None` (a drag, a placement, the editor's
    /// own undo) is its own step.
    fn set_ecad_block(&mut self, key: &str, block: Option<Value>, coalesce_key: Option<&str>) {
        let coalesce_key = coalesce_key.map(|change| format!("{key}:{change}"));
        self.checkpoint(coalesce_key.as_deref());
        self.put_ecad_block(key, block);
    }

    fn put_ecad_block(&mut self, key: &str, block: Option<Value>) {
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert(key.into(), block);
                }
                None => {
                    object.remove(key);
                }
            }
        }
    }
}

// ============================================================================
// Pins and points: a part's symbol pins ARE its declared connection points,
// bound by NAME (eCAD plan decision 8; `brep_kernel` `part_pins` holds the
// rules).
//
// The binding lives HERE, inside the writes, so no caller can edit one side
// without the other: the symbol setters follow a pin edit to the points, and
// every mutator that can add, remove or rename a point follows it to the pins,
// inside the same undo step. Undo, redo and a replaced document need nothing,
// since a snapshot restores both sides together.
//
// Each follow diffs against the RUN BASE: the document the current coalesced
// run started from. Typing `10` over pin `2` is four writes under one eCAD key
// (`2`, ``, `1`, `10`). The two in the middle cannot bind (no label, and a
// label pin 1 already has), so they are HELD: the user's edit stands, the
// other side keeps what the last bound write gave it, and `pin_port_hold`
// names why. Replaying from the base makes the last keystroke decide
// whichever came before it.
// ============================================================================

impl History {
    /// Why the latest pin or port edit was not followed to the other side, in
    /// words that name the label (`two pins are labelled '1'`), or `None`.
    pub fn pin_port_hold(&self) -> Option<&str> {
        self.pin_port_hold.as_deref()
    }

    /// Everything about this part's pins and connection points that does not
    /// pair, by name; `None` for a document without a symbol.
    pub fn pin_point_report(&self) -> Option<brep_kernel::PinPointReport> {
        brep_kernel::pin_point_report(self.document())
    }

    /// Whether a symbol edit has changed the declared points since the history
    /// last ran, clearing it. The engine re-runs when it was set.
    pub(crate) fn take_ports_followed(&mut self) -> bool {
        std::mem::take(&mut self.ports_followed)
    }

    /// Whether the next [`crate::engine_state::EngineState::pump`] will re-run
    /// for a moved point, without clearing it — so the app can hand a document
    /// that is not in front its own parts library before that run is submitted.
    pub fn ports_follow_pending(&self) -> bool {
        self.ports_followed
    }

    /// The document the current coalesced run started from. Right after a
    /// checkpointing write it is the top of the undo stack: [`Self::checkpoint`]
    /// pushes the pre-write state unless the write joins a run, and a run's
    /// first write pushed the state the run started from. Eviction takes the
    /// OLDEST entry, never the top. The stack is never empty after a
    /// checkpoint, so the fallback (the current document, which binds nothing)
    /// is never taken by a follow. Borrowed: a follow runs on every write of a
    /// drag, so the base is compared in place and never copied.
    fn run_base(&self) -> &Value {
        self.undo_stack
            .last()
            .map(|snapshot| &snapshot.request)
            .unwrap_or_else(|| self.document())
    }

    /// Carry the symbol edit between the base (`base`, else the run base) and
    /// now to the connection points and the pads: a renamed pin renames its
    /// point in place (so the point keeps its placement), a removed pin removes
    /// its point, and a new pin gets a new point in the part's first port
    /// group.
    fn follow_pins(&mut self, base: Option<Value>) {
        let mut document = self.document().clone();
        let outcome = {
            let base = base.as_ref().unwrap_or_else(|| self.run_base());
            brep_kernel::follow_pin_edit(base, &mut document)
        };
        self.settle(outcome, document);
    }

    /// Carry a connection-point edit between the run base and now to the pins
    /// and pads. Only an edit that changed the declared points is followed, so
    /// an unrelated edit neither binds nor clears a standing hold.
    fn follow_ports(&mut self) {
        if self.symbol_block().is_none() {
            return;
        }
        if brep_kernel::declared_points(self.run_base()) == brep_kernel::declared_points(self.document()) {
            return;
        }
        let mut document = self.document().clone();
        let outcome = brep_kernel::follow_point_edit(self.run_base(), &mut document);
        self.settle(outcome, document);
    }

    /// Adopt a follow's result, or record why it was held. The write goes
    /// through `document_mut`, not a checkpoint: it belongs to the step the
    /// user's own write just took. The rolled-to step keeps its meaning: a
    /// model rolled to its last feature is still rolled to its last feature
    /// once a port is appended or removed, and one rolled back stays on the
    /// same feature.
    fn settle(&mut self, outcome: Result<Vec<brep_kernel::PinPointChange>, String>, document: Value) {
        let changes = match outcome {
            Ok(changes) => changes,
            Err(reason) => {
                self.pin_port_hold = Some(reason);
                return;
            }
        };
        self.pin_port_hold = None;
        if changes.is_empty() {
            return;
        }
        let moves_a_port = changes.iter().any(|change| {
            use brep_kernel::PinPointChange::*;
            // Exhaustive on purpose: a new kind of change must say whether
            // it touches the geometry a run publishes.
            match change {
                PointRenamed { .. } | PointAdded { .. } | PointRemoved { .. } => true,
                PinRelabelled { .. } | PinAdded { .. } | PinRemoved { .. }
                | PadsRenumbered { .. } | PadsAmbiguous { .. } => false,
            }
        });
        let at_end = self.rollback + 1 >= self.len();
        let rolled_to = self.feature_id(self.rollback());
        *self.document_mut() = document;
        let len = self.len();
        self.rollback = if len == 0 {
            0
        } else if at_end {
            len - 1
        } else {
            rolled_to
                .and_then(|id| self.index_of(&id))
                .unwrap_or_else(|| self.rollback.saturating_sub(1))
                .min(len - 1)
        };
        self.ports_followed |= moves_a_port;
    }
}


// ============================================================================
// Named top-level blocks the APP owns (appended — the document-classes slice).
// The history holds the document as a JSON value, so a key the kernel never
// reads (`documentClass`, a family's `familyTable`, a template's
// `templateInputs`, a member's `familySource`) rides save, open and every undo
// snapshot untouched. These are the doors that change one: the user-edit one
// records an undo step, the other is for a write that is bookkeeping rather
// than an edit (the class a load reads off the file's extension).
// ============================================================================
impl History {
    /// The document's top-level block `key`, if present.
    pub fn document_block(&self, key: &str) -> Option<&Value> {
        self.document().get(key)
    }

    /// Replace (or, with `None`, remove) top-level block `key` as a USER edit:
    /// snapshotted for undo. `coalesce_key` groups a typing run into one step,
    /// namespaced by the block so a run in one never swallows another's.
    pub fn set_document_block(&mut self, key: &str, block: Option<Value>, coalesce_key: Option<&str>) {
        let coalesce_key = coalesce_key.map(|change| format!("block:{key}:{change}"));
        self.checkpoint(coalesce_key.as_deref());
        self.set_document_block_no_undo(key, block);
    }

    /// [`Self::set_document_block`] with no undo step.
    pub fn set_document_block_no_undo(&mut self, key: &str, block: Option<Value>) {
        if let Some(object) = self.document_mut().as_object_mut() {
            match block {
                Some(block) => {
                    object.insert(key.to_string(), block);
                }
                None => {
                    object.remove(key);
                }
            }
        }
    }
}

