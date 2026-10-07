//! Official KiCad symbol → assigned footprint → STEP chain, at one pinned release.
//! Library name segments retain KiCad's function/manufacturer/series hierarchy.
use super::ImportRequest;
use crate::{
    db::{Db, PartSpec},
    model::{DocumentClass, Lifecycle, Origin, User},
    Error,
};
use brep_ecad_core::{kicad, Symbol};
use serde_json::{json, Value};
use std::{collections::HashMap, io::Read};

const TAG: &str = "9.0.9.1";
const LIMIT: u64 = 32 * 1024 * 1024;

fn valid_library(name: &str) -> bool {
    !name.is_empty() && name.len() <= 100 && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Discover every top-level symbol library at the same pinned release used
/// for assets. Pagination is exhausted; no hand-picked library subset.
pub(super) fn libraries(db: &Db) -> Result<Value, Error> {
    let cache = db.root().join("kicad-cache").join(TAG).join("symbol-library-index.json");
    let names: Vec<String> = if cache.is_file() {
        let bytes = std::fs::read(&cache).map_err(Error::internal)?;
        if bytes.len() as u64 > LIMIT { return Err(Error::bad_request("KiCad index exceeds download limit")); }
        serde_json::from_slice(&bytes).map_err(|e| Error::bad_request(e.to_string()))?
    } else {
        let agent = ureq::AgentBuilder::new().timeout(std::time::Duration::from_secs(90)).build();
        let mut names = Vec::new();
        for page in 1..=100 {
            let url = format!("https://gitlab.com/api/v4/projects/kicad%2Flibraries%2Fkicad-symbols/repository/tree?ref={TAG}&per_page=100&page={page}");
            let response = agent.get(&url).call().map_err(|e| Error::bad_request(format!("KiCad library discovery failed: {e}; retry")))?;
            let mut bytes = Vec::new();
            response.into_reader().take(LIMIT + 1).read_to_end(&mut bytes).map_err(Error::internal)?;
            if bytes.len() as u64 > LIMIT { return Err(Error::bad_request("KiCad index exceeds download limit")); }
            let entries: Vec<Value> = serde_json::from_slice(&bytes).map_err(|e| Error::bad_request(e.to_string()))?;
            for entry in &entries {
                if entry["type"] == "blob" {
                    if let Some(name) = entry["path"].as_str().and_then(|p| p.strip_suffix(".kicad_sym")) {
                        if !valid_library(name) { return Err(Error::bad_request(format!("Unsupported upstream library name: {name}"))); }
                        names.push(name.to_string());
                    }
                }
            }
            if entries.len() < 100 { break; }
            if page == 100 { return Err(Error::bad_request("KiCad index pagination exceeds limit; no partial list was imported")); }
        }
        names.sort(); names.dedup();
        if names.is_empty() { return Err(Error::bad_request("KiCad returned no symbol libraries")); }
        std::fs::create_dir_all(cache.parent().unwrap()).map_err(Error::internal)?;
        let temporary = cache.with_extension(format!("{}.tmp", crate::auth::new_id()));
        std::fs::write(&temporary, serde_json::to_vec(&names).map_err(Error::internal)?).map_err(Error::internal)?;
        std::fs::rename(&temporary, &cache).map_err(Error::internal)?;
        names
    };
    if names.is_empty() || names.iter().any(|n| !valid_library(n)) { return Err(Error::bad_request("Invalid cached KiCad library index")); }
    Ok(json!({"release":TAG,"libraries":names}))
}

/// Reclassify legacy imports without touching any document, revision or
/// lifecycle. In particular, user edits and released recipes remain intact.
pub(super) fn repair_taxonomy(db: &Db, root: &str) -> Result<Value, Error> {
    if !root.is_empty() && db.read(|s| crate::catalog::find(&s.categories, root).is_none()) {
        return Err(Error::bad_request("choose an existing catalog root"));
    }
    let candidates = db.read(|s| s.parts.iter().filter(|p| p.document_class == DocumentClass::Normal).cloned().collect::<Vec<_>>());
    let mut parts = Vec::new();
    for part in candidates {
        let Some((library, _)) = part.external_ref.split_once(':').filter(|(name, _)| valid_library(name)) else { continue; };
        let Some(revision) = part.latest().filter(|r| r.origin == Origin::Imported) else { continue; };
        let Some(body) = db.read_document(&revision.document_key(&part.id))? else { continue; };
        let document: Value = serde_json::from_str(&body).map_err(Error::internal)?;
        if document["symbol"]["library_id"].as_str() != Some(part.external_ref.as_str()) { continue; }
        let modern = document["kicadImport"]["library"].as_str() == Some(library)
            && document["kicadImport"]["release"].as_str() == Some(TAG)
            && document["kicadImport"]["sourceHash"].as_str().is_some_and(|h| h.len() == 64 && h.chars().all(|c| c.is_ascii_hexdigit()));
        let legacy = part.description == "KiCad symbol library import";
        if modern || legacy { parts.push((part.id.clone(), library.to_string(), part.category.clone())); }
    }
    let mut changed = 0;
    for (id, library, old) in &parts {
        let category = category(db, root, library)?;
        if old != &category { db.update_part(id, &json!({"category":category}))?; changed += 1; }
    }
    Ok(json!({"parts":parts.len(),"reclassified":changed,"release":TAG}))
}

fn encode(value: &str) -> String {
    value
        .bytes()
        .map(|b| match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                (b as char).to_string()
            }
            _ => format!("%{b:02X}"),
        })
        .collect()
}

fn safe_path(path: &str) -> Result<(), String> {
    if path.is_empty()
        || path.len() > 1000
        || path
            .split('/')
            .any(|s| s.is_empty() || s == "." || s == "..")
        || path
            .chars()
            .any(|c| c.is_control() || matches!(c, '\\' | '$' | ':'))
    {
        return Err(format!("unsupported KiCad repository path: {path}"));
    }
    Ok(())
}

/// URLs and repository names are fixed here; upstream model paths cannot turn
/// the import into arbitrary HTTP requests or escape the on-disk cache.
struct Files<'a> {
    db: &'a Db,
    agent: ureq::Agent,
    results: HashMap<String, Result<Option<String>, String>>,
}
impl Files<'_> {
    fn read(&mut self, repo: &str, path: &str) -> Result<Option<String>, String> {
        safe_path(path)?;
        let key = format!("{repo}/{path}");
        if let Some(result) = self.results.get(&key) {
            return result.clone();
        }
        let result = self.fetch(repo, path);
        self.results.insert(key, result.clone());
        result
    }
    fn fetch(&self, repo: &str, path: &str) -> Result<Option<String>, String> {
        let cached = self
            .db
            .root()
            .join("kicad-cache")
            .join(TAG)
            .join(repo)
            .join(path);
        if cached.is_file() {
            let bytes = std::fs::read(&cached).map_err(|e| e.to_string())?;
            if bytes.len() as u64 > LIMIT {
                return Err("cached KiCad file exceeds 32 MiB".into());
            }
            return String::from_utf8(bytes)
                .map(Some)
                .map_err(|e| e.to_string());
        }
        let url = format!("https://gitlab.com/api/v4/projects/kicad%2Flibraries%2F{repo}/repository/files/{}/raw?ref={TAG}", encode(path));
        let response = match self.agent.get(&url).call() {
            Ok(response) => response,
            Err(ureq::Error::Status(404, _)) => return Ok(None),
            Err(error) => {
                return Err(format!(
                    "KiCad download failed ({path}): {error}; retry the import"
                ))
            }
        };
        let mut bytes = Vec::new();
        response
            .into_reader()
            .take(LIMIT + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() as u64 > LIMIT {
            return Err(format!("KiCad file exceeds 32 MiB: {path}"));
        }
        let text = String::from_utf8(bytes).map_err(|e| e.to_string())?;
        std::fs::create_dir_all(cached.parent().unwrap()).map_err(|e| e.to_string())?;
        let temporary = cached.with_extension(format!("{}.tmp", crate::auth::new_id()));
        std::fs::write(&temporary, &text).map_err(|e| e.to_string())?;
        if let Err(error) = std::fs::rename(&temporary, &cached) {
            let _ = std::fs::remove_file(&temporary);
            return Err(error.to_string());
        }
        Ok(Some(text))
    }
}

/// Reuse nodes by parent and name, rather than flattening all libraries into
/// Electrical / Components. Keep the exact nickname in external_ref and source.
fn category(db: &Db, root: &str, library: &str) -> Result<String, Error> {
    let mut parent = root.to_string();
    for name in std::iter::once("KiCad").chain(library.split('_').filter(|s| !s.is_empty())) {
        parent = db.mutate(|state| {
            if let Some(existing) = state.categories.iter().find(|c| {
                c.parent.eq_ignore_ascii_case(&parent) && c.name.eq_ignore_ascii_case(name)
            }) {
                return Ok(existing.id.clone());
            }
            let base = format!(
                "kicad-{}",
                &crate::auth::content_hash(&format!("{parent}/{name}"))[..20]
            );
            let mut id = base.clone();
            let mut suffix = 1;
            while crate::catalog::find(&state.categories, &id).is_some() {
                id = format!("{base}-{suffix}");
                suffix += 1;
            }
            state.categories.push(crate::model::Category {
                id: id.clone(),
                name: name.into(),
                parent: parent.clone(),
                attributes: Vec::new(),
                created_at: crate::db::now(),
            });
            crate::catalog::check_tree(&state.categories)?;
            Ok(id)
        })?;
    }
    Ok(parent)
}

fn model_path(path: &str) -> Result<String, String> {
    let path = path
        .strip_prefix("${KICAD9_3DMODEL_DIR}/")
        .or_else(|| path.strip_prefix("${KICAD8_3DMODEL_DIR}/"))
        .or_else(|| path.strip_prefix("${KICAD7_3DMODEL_DIR}/"))
        .or_else(|| path.strip_prefix("${KICAD6_3DMODEL_DIR}/"))
        .or_else(|| path.strip_prefix("${KISYS3DMOD}/"))
        .unwrap_or(path);
    safe_path(path)?;
    if !path.ends_with(".step") {
        return Err(format!("no STEP solid model for {path}"));
    }
    Ok(path.to_string())
}

/// The shared footprint parser retains one model for its PCB type. A library
/// part can have several, so extract every top-level model expression and let
/// that same parser read its placement. Respect quoted parentheses and escapes.
fn footprint_models(text: &str) -> Result<Vec<brep_ecad_core::board::Model>, String> {
    let bytes = text.as_bytes();
    let mut depth = 0;
    let mut quoted = false;
    let mut escaped = false;
    let mut start = None;
    let mut models = Vec::new();
    for (i, &byte) in bytes.iter().enumerate() {
        if quoted {
            if escaped {
                escaped = false;
            } else if byte == b'\\' {
                escaped = true;
            } else if byte == b'"' {
                quoted = false;
            }
            continue;
        }
        match byte {
            b'"' => quoted = true,
            b'(' => {
                if depth == 1 {
                    let head = text[i + 1..].trim_start();
                    if head
                        .strip_prefix("model")
                        .is_some_and(|tail| tail.starts_with(char::is_whitespace))
                    {
                        start = Some(i);
                    }
                }
                depth += 1;
            }
            b')' => {
                depth -= 1;
                if depth == 1 {
                    if let Some(start) = start.take() {
                        let (footprint, _) = kicad::import_footprint(&format!(
                            "(footprint \"model\" {})",
                            &text[start..=i]
                        ))?;
                        if let Some(model) = footprint.model {
                            models.push(model);
                        }
                    }
                }
            }
            _ => {}
        }
    }
    Ok(models)
}

fn feature(
    text: &str,
    model: &brep_ecad_core::board::Model,
    path: &str,
    id: &str,
) -> Result<Value, String> {
    let (bodies, appearances) = brep_kernel::import_step_with_appearance(text)?;
    let transform = brep_kernel::AffineTransform::new(kicad::model_placement(model))?;
    let mirrored = transform.determinant3() < 0.0;
    let bodies = bodies
        .iter()
        .map(|b| brep_kernel::transform_brep(b, transform, mirrored))
        .collect::<Result<Vec<_>, _>>()?;
    let _isolation = brep_kernel::IsolatedSceneMetadata::begin();
    let payload = brep_kernel::native_import_payload_with_appearance(id, &bodies, &appearances)?;
    Ok(
        json!({"type":"IMPORT3D","inputParams":{"id":id,"nativeBrep":payload},
        "persistentData":{"kicadModel":{"path":model.path,"stepFile":path,"offset":model.offset,"scale":model.scale,"rotation":model.rotation}}}),
    )
}

/// Hash only importer-owned source content; changes to that content prevent a
/// retry from replacing user edits. The old symbol-only format is matched exactly.
fn source_hash(document: &Value) -> String {
    let mut source = document.clone();
    if let Some(object) = source.as_object_mut() {
        object.remove("partAttributes");
        object.remove("kicadImport");
        object.remove("thumbnail");
    }
    source.sort_all_objects();
    crate::auth::content_hash(&serde_json::to_string(&source).unwrap())
}

fn checked_body(
    db: &Db,
    request: &ImportRequest,
    document: &Value,
    attributes: &Value,
) -> Result<String, Error> {
    let mut document = document.clone();
    document["partAttributes"] = attributes.clone();
    document["kicadImport"] =
        json!({"release":TAG,"library":request.library,"sourceHash":source_hash(&document)});
    let body = serde_json::to_string(&document).map_err(Error::internal)?;
    if body.len() as u64 > db.security().config.document_limit() {
        return Err(Error::bad_request(
            "KiCad part exceeds the document size limit",
        ));
    }
    Ok(body)
}

fn install(
    db: &Db,
    user: &User,
    request: &ImportRequest,
    symbol: &Symbol,
    document: Value,
) -> Result<&'static str, Error> {
    let existing = db.read(|s| {
        s.parts
            .iter()
            .find(|p| p.part_type == request.part_type && p.external_ref == symbol.library_id)
            .cloned()
    });
    let mut outcome = "created";
    let part = if let Some(part) = existing {
        // Classification is intentionally updated even for completed imports.
        if part.category != request.category {
            db.update_part(&part.id, &json!({"category":request.category}))?;
        }
        outcome = "updated";
        part
    } else {
        // Bound the complete document before allocating a number/part. A counter's
        // maximum number is at least as long as the number it will allocate.
        let number = db
            .read(|s| {
                s.part_type(&request.part_type)
                    .map(|t| t.format_number(t.capacity()))
            })
            .ok_or_else(|| Error::bad_request("choose a part type"))?;
        checked_body(
            db,
            request,
            &document,
            &json!({"Part_Number":number,"Description":symbol.description}),
        )?;
        db.create_part_with(
            user,
            &PartSpec {
                part_type: request.part_type.clone(),
                name: symbol
                    .library_id
                    .rsplit(':')
                    .next()
                    .unwrap_or(&symbol.library_id)
                    .into(),
                category: request.category.clone(),
                description: symbol.description.clone(),
                document_class: DocumentClass::Normal,
                external_ref: symbol.library_id.clone(),
                origin: Origin::Imported,
                ..PartSpec::default()
            },
        )?
    };
    let mut attributes = json!({"Part_Number":part.number,"Description":part.description});
    let mut revision = part
        .latest()
        .ok_or_else(|| Error::conflict("import part has no revision"))?
        .clone();
    if revision.origin != Origin::Imported {
        return Ok("skipped");
    }
    if !revision.content_hash.is_empty() {
        let old: Value = serde_json::from_str(
            &db.read_document(&revision.document_key(&part.id))?
                .ok_or_else(|| Error::conflict("import document missing"))?,
        )
        .map_err(Error::internal)?;
        if let Some(old_attributes) = old.get("partAttributes") {
            attributes = old_attributes.clone();
        }
        let hash = source_hash(&old);
        let legacy = json!({"features":[],"expressions":"","symbol":symbol});
        // The former starter installer touched familyTable.rows even on symbols,
        // materializing this otherwise inert field in its old documents.
        let mut legacy_family = legacy.clone();
        legacy_family["familyTable"] = json!({"rows":null});
        let unchanged = old["kicadImport"]["sourceHash"].as_str() == Some(hash.as_str())
            || hash == source_hash(&legacy)
            || hash == source_hash(&legacy_family);
        let would_lose_assets = (old.get("pads").is_some() && document.get("pads").is_none())
            || old["features"].as_array().map_or(0, Vec::len)
                > document["features"].as_array().map_or(0, Vec::len);
        if would_lose_assets
            || hash == source_hash(&document)
            || revision.origin != Origin::Imported
            || !unchanged
            || revision.lock.is_some()
            || revision.lifecycle == Lifecycle::InReview
        {
            return Ok("skipped");
        }
        // Preserve the old revision and all released bytes. Enrichment becomes
        // a new imported draft; it never bypasses approval/release workflows.
        checked_body(db, request, &document, &attributes)?;
        let created = db.create_revision_from(user, &part.id, "", Origin::Imported)?;
        // create_revision_from copies the predecessor document after returning
        // its initial metadata; use the persisted copied hash for the guarded write.
        revision = db
            .read(|s| {
                s.part(&part.id)
                    .and_then(|p| p.revision(&created.id))
                    .cloned()
            })
            .ok_or_else(|| Error::conflict("import revision disappeared"))?;
    }
    let body = checked_body(db, request, &document, &attributes)?;
    db.checkout(user, &part.id, &revision.id, "setup-wizard")?;
    let written = db.write_import_document(
        user,
        &part.id,
        &revision.id,
        &revision.content_hash,
        &body,
        &json!({"uses":[]}),
    );
    let checked_in = db.checkin(user, &part.id, &revision.id, false);
    written?;
    checked_in?;
    Ok(outcome)
}

pub(super) fn import(db: &Db, user: &User, request: &ImportRequest) -> Result<Value, Error> {
    let library = request.library.trim();
    if !valid_library(library) {
        return Err(Error::bad_request(
            "enter a KiCad library name, such as Device or Timer",
        ));
    }
    let mut files = Files {
        db,
        agent: ureq::AgentBuilder::new()
            .timeout(std::time::Duration::from_secs(90))
            .build(),
        results: HashMap::new(),
    };
    let text = files
        .read("kicad-symbols", &format!("{library}.kicad_sym"))
        .map_err(Error::bad_request)?
        .ok_or_else(|| Error::bad_request("KiCad symbol library not found"))?;
    let report = kicad::import_library(&text, library).map_err(Error::bad_request)?;
    let mut request = request.clone();
    request.library = library.into();
    request.category = category(db, &request.category, library)?;
    let mut warnings = report.warnings;
    let mut counts = json!({"created":0,"updated":0,"skipped":0,"excluded":0,"symbols":report.symbols.len(),"footprints":0,"models":0,"needs_footprint":0,"without_model":0,"footprints_without_model_reference":0,"model_references":0,"model_reference_errors":0,"installed_footprints":0,"installed_models":0});
    let mut failed = Vec::new();
    let mut excluded = Vec::new();
    let mut models: HashMap<String, Result<Value, String>> = HashMap::new();
    for symbol in report.symbols {
        let mut document = json!({"features":[],"expressions":"","symbol":symbol});
        let assets = (|| -> Result<(), String> {
            let (library, name) = match kicad::footprint_link(&symbol) {
                kicad::FootprintLink::Named { library, name } => (library, name),
                kicad::FootprintLink::Chosen { .. } => {
                    counts["needs_footprint"] =
                        json!(counts["needs_footprint"].as_u64().unwrap() + 1);
                    return Ok(());
                }
            };
            safe_path(&library)?;
            safe_path(&name)?;
            if library.contains('/') || name.contains('/') {
                return Err("footprint nickname must not contain a path".into());
            }
            let path = format!("{library}.pretty/{name}.kicad_mod");
            let text = files
                .read("kicad-footprints", &path)?
                .ok_or_else(|| format!("assigned footprint not found: {library}:{name}"))?;
            let (footprint, notes) = kicad::import_footprint(&text)?;
            warnings.extend(
                notes
                    .into_iter()
                    .map(|n| format!("{}: {n}", symbol.library_id)),
            );
            document["pads"] = json!(footprint);
            document["ports"] = json!([{"name":"Pins","purpose":"pcb","points":symbol.pins.iter().filter({ let mut seen = std::collections::BTreeSet::new(); move |pin| seen.insert(pin.number.clone()) }).map(|pin| {
                let pad = footprint.pads.iter().find(|p| p.number == pin.number);
                let position = pad.map(|p| [p.at.x as f64 / 1000., -p.at.y as f64 / 1000., 0.]).unwrap_or([0.,0.,0.]);
                json!({"name":pin.number,"transform":{"position":position,"rotationEuler":[0,-90,0]}})
            }).collect::<Vec<_>>()}]);
            counts["footprints"] = json!(counts["footprints"].as_u64().unwrap() + 1);
            let mut notes = Vec::new();
            let references = footprint_models(&text)?;
            if references.is_empty() {
                counts["footprints_without_model_reference"] = json!(
                    counts["footprints_without_model_reference"]
                        .as_u64()
                        .unwrap()
                        + 1
                );
            }
            counts["model_references"] =
                json!(counts["model_references"].as_u64().unwrap() + references.len() as u64);
            for (index, model) in references.iter().enumerate() {
                let result = (|| -> Result<Value, String> {
                    let path = model_path(&model.step_path())?;
                    let id = format!("KiCadModel{}", index + 1);
                    let key = serde_json::to_string(&(path.clone(), model, &id)).unwrap();
                    models
                        .entry(key)
                        .or_insert_with(|| {
                            let text = files
                                .read("kicad-packages3D", &path)?
                                .ok_or_else(|| format!("STEP model not found: {path}"))?;
                            feature(&text, model, &path, &id)
                        })
                        .clone()
                })();
                match result {
                    Ok(feature) => document["features"].as_array_mut().unwrap().push(feature),
                    Err(error) => {
                        counts["model_reference_errors"] =
                            json!(counts["model_reference_errors"].as_u64().unwrap() + 1);
                        notes.push(error);
                    }
                }
            }
            if !document["features"].as_array().unwrap().is_empty() {
                counts["models"] = json!(counts["models"].as_u64().unwrap() + 1);
            }
            if !notes.is_empty() {
                return Err(notes.join("; "));
            }
            Ok(())
        })();
        if document["features"].as_array().unwrap().is_empty() {
            counts["without_model"] = json!(counts["without_model"].as_u64().unwrap() + 1);
        }
        if let Err(error) = assets {
            warnings.push(format!("{}: {error}", symbol.library_id));
        }
        let has_footprint = document.get("pads").is_some();
        let has_model = !document["features"].as_array().unwrap().is_empty();
        // Only complete symbol → footprint → usable solid chains enter PLM.
        // Check before install allocates a part number or changes an existing part.
        if !has_footprint || !has_model {
            counts["excluded"] = json!(counts["excluded"].as_u64().unwrap() + 1);
            excluded.push(json!({"name":symbol.library_id,
                "missing_footprint":!has_footprint,"missing_model":!has_model}));
            continue;
        }
        match install(db, user, &request, &symbol, document) {
            Ok(key) => {
                counts[key] = json!(counts[key].as_u64().unwrap() + 1);
                if key != "skipped" {
                    if has_footprint {
                        counts["installed_footprints"] =
                            json!(counts["installed_footprints"].as_u64().unwrap() + 1);
                    }
                    if has_model {
                        counts["installed_models"] =
                            json!(counts["installed_models"].as_u64().unwrap() + 1);
                    }
                }
            }
            Err(error) => failed.push(json!({"name":symbol.library_id,"error":error.to_string()})),
        }
    }
    counts["excluded_parts"] = json!(excluded);
    counts["failed"] = json!(failed);
    counts["warnings"] = json!(warnings);
    counts["category"] = json!(request.category);
    counts["release"] = json!(TAG);
    Ok(counts)
}

