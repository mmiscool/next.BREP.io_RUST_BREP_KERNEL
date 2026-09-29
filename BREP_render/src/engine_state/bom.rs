use super::*;
use serde_json::Value;

// ===========================================================================
// BOM export — the parts list straight off the MAIN-SIDE parts library + live
// component projection, both adopted from the last applied run's reply
// (`pipeline::AssemblySync`). One row per parts-library entry: `{partName,
// sourceKey, quantity}` with quantity = live ACOMP instance count. A
// sub-assembly is ONE row at this level — its internal parts are its own
// document's business (the rigid-nesting model). Then one row per harness
// WIRE (plan decision 6: each wire is its own BOM line): its stock part
// number as the part name, quantity 1, and `{connectionId, mfQty, status}` —
// the cut length, or no number and the reason ([`super::WireBomLine`]).
// Exported as CSV and JSON through the file dialog's Export modal;
// deliberately small: no per-configuration quantities, no extra columns, no
// localization.
// ===========================================================================

/// One BOM row: a parts-library entry + its live instance count, or a wire.
/// Field order IS the exported JSON key order (serde serializes structs in
/// declaration order), so a part row stays `{partName, sourceKey, quantity}`
/// and a wire row adds `{connectionId, mfQty, status}`.
#[derive(serde::Serialize)]
struct BomRow {
    #[serde(rename = "partName")]
    part_name: String,
    #[serde(rename = "sourceKey")]
    source_key: String,
    quantity: usize,
    #[serde(flatten, skip_serializing_if = "Option::is_none")]
    wire: Option<WireFields>,
}

/// A wire row's own fields. `mfQty` is `null` — never absent, never stale —
/// when the run on hand cannot give a cut length; `status` says why.
#[derive(serde::Serialize)]
struct WireFields {
    #[serde(rename = "connectionId")]
    connection_id: String,
    #[serde(rename = "mfQty")]
    mf_qty: Option<f64>,
    status: String,
}

/// The CSV header: the part columns, then the wire columns (blank on a part
/// row). One header for every document, so a reader never guesses the shape.
const BOM_CSV_HEADER: &str = "partName,sourceKey,quantity,connectionId,mfQty,status\n";

/// Quote a CSV field only when it needs it (comma / quote / CR / LF),
/// doubling embedded quotes — minimal RFC-4180 so a part name or store path
/// with a comma can never shear a row.
fn csv_field(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

impl EngineState {
    /// The BOM rows: parts in parts-library order (BTreeMap ⇒ alphabetical
    /// part name — deterministic), then wires in the harness block's order.
    /// Quantity counts component RECORDS (instances), not member solids, so a
    /// multi-body part is still one per placement; a wire is always one. Errs
    /// (`"no components or harness wires in the document"`) when there is
    /// neither — a harness-only document is a BOM of wires.
    fn bom_rows(&mut self) -> Result<Vec<BomRow>, String> {
        let wires = self.wire_bom_lines();
        if self.assembly_components.is_empty() && wires.is_empty() {
            return Err("no components or harness wires in the document".into());
        }
        let mut rows: Vec<BomRow> = Vec::new();
        if !self.assembly_components.is_empty() {
            let library: std::collections::BTreeMap<String, Value> =
                serde_json::from_str(&brep_kernel::parts_library_json())
                    .map_err(|error| format!("parts library unreadable: {error}"))?;
            rows.extend(
                library
                    .into_iter()
                    .map(|(part_name, entry)| {
                        let quantity = self
                            .assembly_components
                            .iter()
                            .filter(|record| record.part_name == part_name)
                            .count();
                        BomRow {
                            source_key: entry
                                .get("sourceKey")
                                .and_then(Value::as_str)
                                .unwrap_or_default()
                                .to_string(),
                            part_name,
                            quantity,
                            wire: None,
                        }
                    })
                    // The kernel GCs zero-instance entries at the end of every
                    // run; the filter keeps the export honest mid-mutation
                    // regardless.
                    .filter(|row| row.quantity > 0),
            );
        }
        rows.extend(wires.into_iter().map(|line| BomRow {
            part_name: line.stock_part_number,
            source_key: String::new(),
            quantity: 1,
            wire: Some(WireFields {
                connection_id: line.connection_id,
                mf_qty: line.mf_qty,
                status: line.state.as_str().to_string(),
            }),
        }));
        Ok(rows)
    }

    /// Export the BOM as CSV: the exact [`BOM_CSV_HEADER`], one line per row,
    /// LF endings. A cut length is written to three decimals; a wire without
    /// one leaves `mfQty` empty and names the reason in `status`.
    pub fn export_bom_csv(&mut self) -> Result<String, String> {
        let mut out = String::from(BOM_CSV_HEADER);
        for row in self.bom_rows()? {
            let (connection, mf_qty, status) = match &row.wire {
                Some(wire) => (
                    csv_field(&wire.connection_id),
                    wire.mf_qty.map(|qty| format!("{qty:.3}")).unwrap_or_default(),
                    wire.status.clone(),
                ),
                None => (String::new(), String::new(), String::new()),
            };
            out.push_str(&format!(
                "{},{},{},{},{},{}\n",
                csv_field(&row.part_name),
                csv_field(&row.source_key),
                row.quantity,
                connection,
                mf_qty,
                status
            ));
        }
        Ok(out)
    }

    /// Export the BOM as a JSON array of the same records — the CSV sibling of
    /// [`Self::export_bom_csv`], with the cut length at full precision.
    pub fn export_bom_json(&mut self) -> Result<String, String> {
        serde_json::to_string(&self.bom_rows()?)
            .map_err(|error| format!("BOM serialize: {error}"))
    }

    // ======================================================================
    // BOM ATTRIBUTES — the editable columns behind the BOM panel
    //
    // Two stores, because the data has two lifetimes:
    //
    // * PART attributes (Part Number, Material, Mass…) describe the PART, so
    //   they live on the part's OWN document, under the top-level
    //   [`PART_ATTRIBUTES`] key: `partsLibrary[part].document.partAttributes`.
    //   Being on the part document means they travel WITH the part — the
    //   write-through lane saves that same document back to its `sourceKey`,
    //   so opening the part standalone shows the same Part Number. `History`
    //   keeps unknown top-level document keys verbatim (`from_request_json`
    //   parses into a `Value` and only lifts out the keys it owns), so the key
    //   round-trips through save/open with no format work.
    //
    // * OCCURRENCE attributes (Item Number, Reference Designator, Find
    //   Number…) describe ONE PLACEMENT, so they live on the placing ACOMP
    //   feature, under [`OCCURRENCE_ATTRIBUTES`] —
    //   `feature.inputParams.bom`. NESTED rather than flat so a user-added
    //   custom column can never collide with a schema param (`isFixed`,
    //   `partName`, `transform`); the ACOMP builder reads its four keys by
    //   name and ignores the rest, and the solver write-back fold
    //   (`assembly_apply_document_json`) INSERTS `transform`/`isFixed` into the
    //   existing params object rather than rebuilding it, so a solve never
    //   strips this.
    //
    // Quantity is deliberately absent from both: it is DERIVED (how many
    // occurrences a packed row rolls up), never stored, so it can never
    // disagree with the model.
    // ======================================================================

    /// Read a part's attribute record (`{}` when the part has none / is
    /// unknown). Never errs — a BOM row for a part mid-import simply shows
    /// blanks.
    pub fn part_attributes(&self, part_name: &str) -> Value {
        self.history
            .parts_library()
            .get(part_name)
            .and_then(|entry| entry.get("document"))
            .and_then(|document| document.get(PART_ATTRIBUTES))
            .filter(|value| value.is_object())
            .cloned()
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()))
    }

    /// How many PMI annotations a PART's own document carries, summed over its
    /// views. `0` for a part with no PMI block and for an unknown part.
    ///
    /// A STEP assembly import puts a part-scoped dimension or tolerance on the
    /// PART (once, however many instances place it — the exporter's own rule),
    /// so it is not in the open document's PMI block and the PMI panel, which
    /// shows that block, never mentions it. The BOM is where a reader sees it:
    /// this is the parts list saying "the bolt is toleranced", with the
    /// annotations themselves one Open Part away.
    pub fn part_pmi_count(&self, part_name: &str) -> usize {
        self.history
            .parts_library()
            .get(part_name)
            .and_then(|entry| entry.get("document"))
            .and_then(|document| document.get("pmi"))
            .and_then(|pmi| pmi.get("views"))
            .and_then(Value::as_array)
            .map(|views| {
                views
                    .iter()
                    .filter_map(|view| view.get("annotations").and_then(Value::as_array))
                    .map(|annotations| annotations.len())
                    .sum()
            })
            .unwrap_or(0)
    }

    /// A part's `(sourceKey, sourceSignature)` — what the app's write-through
    /// lane needs to decide whether the file on disk is still the one this
    /// entry was built from. `None` for an unknown part; the key is returned
    /// even when EMPTY (embedded-only), because "" is exactly what the
    /// write-through lane checks for.
    pub fn part_source(&self, part_name: &str) -> Option<(String, String)> {
        let entry = self.history.parts_library().get(part_name)?;
        Some((
            entry
                .get("sourceKey")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_string(),
            entry
                .get("sourceSignature")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_string(),
        ))
    }

    /// A part's embedded document as text — the payload the app writes through
    /// to the part's `sourceKey` after an attribute edit.
    pub fn part_document_json(&self, part_name: &str) -> Option<String> {
        self.history
            .parts_library()
            .get(part_name)
            .and_then(|entry| entry.get("document"))
            .map(|document| document.to_string())
    }

    /// Write ONE part attribute. `Value::Null` (or an empty string) REMOVES the
    /// key, so clearing a cell leaves no `""` litter in the saved document.
    ///
    /// # Why this is not just a document edit
    ///
    /// The document's `partsLibrary` block is a MIRROR of the kernel's
    /// main-side store, not the truth: a run that heals or GCs the store writes
    /// its result back over the block (`apply_run_output`), and the per-run
    /// request does not carry the block at all ([`History::prefix_request`]
    /// omits it). So a change written only into the block is erased by the next
    /// such run. This therefore
    /// writes BOTH: the kernel store (via `refresh_library_entry`, which is the
    /// same door edit-in-place and update-components use) and the document
    /// block + undo checkpoint. [`Self::undo`] re-installs the rewound block
    /// into the store, which is what makes the pair rewind together.
    ///
    /// The `snapshot` is KEPT: an attribute is not geometry, so there is
    /// nothing to re-evaluate. `refresh_library_entry` sees that the new
    /// document builds the same part (`brep_kernel::same_build`) and leaves the
    /// entry clean, so the rerun serves every instance from the snapshot. Before
    /// it did, each attribute edit re-executed the whole part: 23 s for a Mass
    /// edit on a herringbone gear. The panel still commits a text cell on
    /// focus-loss rather than per keystroke, since each commit is a run.
    pub fn set_part_attribute(
        &mut self,
        part_name: &str,
        key: &str,
        value: Value,
    ) -> Result<(), String> {
        if key.is_empty() {
            return Err("part attribute: empty key".to_string());
        }
        let mut block = self.history.parts_library().clone();
        let entry = block
            .get_mut(part_name)
            .ok_or_else(|| format!("no parts-library entry '{part_name}'"))?;
        let document = entry
            .get_mut("document")
            .filter(|value| value.is_object())
            .ok_or_else(|| format!("part '{part_name}': malformed document"))?;
        write_attribute(document, PART_ATTRIBUTES, key, value)?;
        let document_text = document.to_string();
        let signature = super::document_signature(&document_text);
        entry["sourceSignature"] = Value::String(signature.clone());

        // The kernel store is the block's authority — write it there too, or
        // the next run that touches the library mirrors the OLD entry back over
        // this edit.
        brep_kernel::refresh_library_entry_impl(part_name, &signature, &document_text)
            .map_err(|error| format!("part '{part_name}': {error}"))?;
        self.history
            .set_parts_library_edited(block, Some(&format!("partattr:{part_name}:{key}")));
        self.rerun_history();
        Ok(())
    }

    // --- The document's OWN part attributes -------------------------------
    //
    // The same `partAttributes` record, on the document you have OPEN rather
    // than on a library entry's embedded one. A part document IS a part, so it
    // carries the BOM data of the part it describes — and an assembly document
    // does too, because a rigidly nested assembly is one BOM row in its parent.
    // The toolbar's Properties dialog is the door; the BOM panel's part columns
    // are the other one, onto the same key of a different document.

    /// The open document's own attribute record (`{}` when it has none).
    /// Never errs — a document that was never annotated simply reads blank.
    pub fn document_part_attributes(&self) -> Value {
        self.history
            .part_attributes_block()
            .filter(|value| value.is_object())
            .cloned()
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()))
    }

    /// Write ONE attribute on the open document. `Value::Null` (or an empty
    /// string) REMOVES the key, and the last key takes the empty record with
    /// it — [`write_attribute`]'s rules, shared with the library-entry lane, so
    /// a document that was never annotated serializes exactly as before.
    ///
    /// Unlike [`Self::set_part_attribute`] there is no kernel store to keep in
    /// step: this record belongs to the document in hand, not to a
    /// parts-library mirror, so the write is document + undo checkpoint and
    /// nothing else. The re-run that follows is not for geometry — the history
    /// is unchanged, so every feature is a cache hit — it is what moves the
    /// applied-run generation, which is what refreshes the tab's dirty dot.
    pub fn set_document_part_attribute(
        &mut self,
        key: &str,
        value: Value,
    ) -> Result<(), String> {
        if key.is_empty() {
            return Err("part attribute: empty key".to_string());
        }
        // Rebuild the record through the SHARED writer by handing it an owner
        // shaped like the document, so create / clear / drop-empty behave
        // identically on both doors rather than being written twice.
        let mut owner = Value::Object(serde_json::Map::new());
        if let Some(block) = self.history.part_attributes_block() {
            let block = block.clone();
            if let Some(object) = owner.as_object_mut() {
                object.insert(PART_ATTRIBUTES.to_string(), block);
            }
        }
        write_attribute(&mut owner, PART_ATTRIBUTES, key, value)?;
        let block = owner
            .as_object_mut()
            .and_then(|object| object.remove(PART_ATTRIBUTES));
        self.history
            .set_part_attributes_block(block, Some(&format!("docpartattr:{key}")));
        self.rerun_history();
        Ok(())
    }

    /// Read ONE occurrence's attribute record (`{}` when it has none).
    pub fn occurrence_attributes(&self, component_id: &str) -> Value {
        self.history
            .index_of(component_id)
            .and_then(|index| self.history.feature_params(index))
            .and_then(|params| params.get(OCCURRENCE_ATTRIBUTES).cloned())
            .filter(Value::is_object)
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()))
    }

    /// Every ACOMP's `occurrenceAttributes` in ONE pass over the history,
    /// keyed by component id. The BOM reads all of them every frame, and the
    /// per-id [`Self::occurrence_attributes`] resolves its id with a linear
    /// `index_of` scan — once per row that is quadratic in the component count
    /// (70 ms a frame at 633 components). A component with no attributes maps
    /// to an empty object, so a lookup here reads exactly what the per-id
    /// accessor answers.
    pub fn occurrence_attributes_all(&self) -> std::collections::HashMap<String, Value> {
        (0..self.history.len())
            .filter(|&index| {
                self.history
                    .feature_type(index)
                    .is_some_and(|ty| super::components::is_acomp_feature_type(&ty))
            })
            .filter_map(|index| {
                let id = self.history.feature_id(index)?;
                let attributes = self
                    .history
                    .feature_params(index)
                    .and_then(|params| params.get(OCCURRENCE_ATTRIBUTES).cloned())
                    .filter(Value::is_object)
                    .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
                Some((id, attributes))
            })
            .collect()
    }

    /// Write ONE occurrence attribute across `component_ids` — ONE undo step
    /// however many ids there are.
    ///
    /// The fan-out lane: a PACKED BOM row rolls up every occurrence of a part
    /// whose occurrence data matches, and editing that row's cell must apply to
    /// all of them. One id (the unpacked case) is the same call with a
    /// one-element slice, so there is no second code path to keep in step.
    /// `Value::Null` / `""` removes the key.
    pub fn set_occurrence_attribute(
        &mut self,
        component_ids: &[String],
        key: &str,
        value: Value,
    ) -> Result<(), String> {
        if key.is_empty() {
            return Err("occurrence attribute: empty key".to_string());
        }
        let mut edits: Vec<(String, Value)> = Vec::with_capacity(component_ids.len());
        for id in component_ids {
            let index = self
                .history
                .index_of(id)
                .ok_or_else(|| format!("no component feature '{id}'"))?;
            let mut params = self
                .history
                .feature_params(index)
                .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
            if !params.is_object() {
                return Err(format!("component '{id}': malformed inputParams"));
            }
            write_attribute(&mut params, OCCURRENCE_ATTRIBUTES, key, value.clone())?;
            edits.push((id.clone(), params));
        }
        // The BOM owns eCAD reference designators. Validate and rename their
        // snapshots in the same document checkpoint as the occurrence edits.
        if key == "Reference_Designator" {
            let mut document: Value = serde_json::from_str(&self.history.request_json())
                .map_err(|e| e.to_string())?;
            let mut renames = Vec::new();
            for (id, params) in &edits {
                let feature = document["features"].as_array_mut().unwrap().iter_mut()
                    .find(|f| f["inputParams"]["id"].as_str() == Some(id)).unwrap();
                let old = feature["inputParams"][OCCURRENCE_ATTRIBUTES][key].as_str().unwrap_or("").to_owned();
                let part = params["partName"].as_str().unwrap_or("").to_owned();
                let new = params[OCCURRENCE_ATTRIBUTES][key].as_str().unwrap_or("").trim().to_owned();
                renames.push((id.clone(), part, old, if new.is_empty() { id.clone() } else { new }));
                feature["inputParams"] = params.clone();
            }
            // A sheet component's label follows the designator of the
            // occurrence it was placed from — INCLUDING a nested one, whose
            // label is the `/`-joined PATH of designators (`A1/J1`) and whose
            // occurrence is a CHAIN (`ACOMP9:ACOMP1`). The BOM addresses
            // top-level features alone (`History::index_of`), so the segment a
            // row owns is the FIRST one, and the rest of the path is the part
            // document's, untouched here.
            let relabel = |instance: Option<&str>, part_key: &str, reference: &str| -> Option<String> {
                renames.iter().find_map(|(id, part, old, new)| match instance {
                    // The chain is the identity and the key is not consulted:
                    // a nested component's key is a key of ITS parent's
                    // library and can never equal this row's part.
                    Some(chain) if chain == id => Some(new.clone()),
                    // A label with NO path has no segment this row owns, and
                    // that is a real shape: `Document::relink` binds a
                    // component to a nested device and deliberately keeps what
                    // it is called. Renaming the outer occurrence must not
                    // overwrite the whole label with its designator.
                    Some(chain) if chain.strip_prefix(id.as_str()).is_some_and(|rest| rest.starts_with(':')) =>
                        reference.split_once('/').map(|(_, rest)| format!("{new}/{rest}")),
                    Some(_) => None,
                    // Legacy, drawn before a component carried an occurrence:
                    // matched by its key and its label, as it always was.
                    None => (part_key == part && reference == old).then(|| new.clone()),
                })
            };
            for block in ["diagram", "pcb"] {
                let Some(components) = document[block]["components"].as_array() else { continue; };
                let affected = components.iter().filter(|c| c["part"].is_object()).any(|component| relabel(
                    component["part"]["instance"].as_str(),
                    component["part"]["key"].as_str().unwrap_or(""),
                    component["reference"].as_str().unwrap_or("")).is_some());
                if !affected { continue; }
                // The schema owner refuses future versions and malformed blocks
                // before either the BOM or the sheet can be changed.
                let mut sheet = brep_ecad_core::Document::from_value(document[block].clone())
                    .map_err(|e| format!("{block}: {e}"))?;
                let expected = if block == "diagram" { brep_ecad_core::DocumentKind::Wiring } else { brep_ecad_core::DocumentKind::Schematic };
                if sheet.kind != expected { return Err(format!("{block}: wrong document kind")); }
                let changes: Vec<_> = sheet.components.iter().filter_map(|component| {
                    let source = component.part.as_ref()?;
                    let new = relabel(source.instance.as_deref(), &source.key, &component.reference)?;
                    Some((component.id, new))
                }).collect();
                for (id, reference) in changes { sheet.set_reference(id, &reference)?; }
                document[block] = sheet.to_value()?;
            }
            self.edit_document_json(&document.to_string())?;
            return Ok(());
        }
        self.update_many_feature_params(&edits)?;
        Ok(())
    }
}

/// The part document's attribute-record key (see the module's BOM-attributes
/// block). A top-level document key, so it rides save/open untouched. Defined
/// by the kernel, which reads the record's `Part_Number` for `PRODUCT.id` on a
/// structured STEP export.
pub use brep_kernel::PART_ATTRIBUTES;

/// The ACOMP `inputParams` attribute-record key.
pub const OCCURRENCE_ATTRIBUTES: &str = "bom";

/// Set (or, for a null/empty value, REMOVE) `record[key]` inside `owner`'s
/// attribute record, creating the record on first write and dropping it again
/// when the last attribute goes — so a document that was never annotated
/// serializes exactly as it did before (the `metadata` field's convention).
fn write_attribute(
    owner: &mut Value,
    record_key: &str,
    key: &str,
    value: Value,
) -> Result<(), String> {
    let object = owner
        .as_object_mut()
        .ok_or_else(|| "attribute owner is not an object".to_string())?;
    let clearing = matches!(&value, Value::Null)
        || matches!(&value, Value::String(text) if text.is_empty());
    if clearing {
        let mut empty = false;
        if let Some(record) = object.get_mut(record_key).and_then(Value::as_object_mut) {
            record.remove(key);
            empty = record.is_empty();
        }
        if empty {
            object.remove(record_key);
        }
        return Ok(());
    }
    let record = object
        .entry(record_key.to_string())
        .or_insert_with(|| Value::Object(serde_json::Map::new()));
    if !record.is_object() {
        *record = Value::Object(serde_json::Map::new());
    }
    record
        .as_object_mut()
        .expect("just normalized to an object")
        .insert(key.to_string(), value);
    Ok(())
}

