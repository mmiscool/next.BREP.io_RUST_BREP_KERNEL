//! Official KiCad symbol → assigned footprint → STEP chain, at one pinned release.
//! Library name segments retain KiCad's function/manufacturer/series hierarchy.
use super::ImportRequest;
use crate::{
    db::{Db, PartSpec},
    model::{DocumentClass, Lifecycle, Origin, User},
    Error,
};
use brep_ecad_core::{
    kicad,
    kicad_git::{self, Repo, TAG},
    Symbol,
};
use serde::Deserialize;
use serde_json::{json, Value};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{atomic::AtomicBool, Mutex},
};

const LIMIT: u64 = 32 * 1024 * 1024;

fn valid_library(name: &str) -> bool {
    !name.is_empty() && name.len() <= 100 && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Where this server keeps KiCad's three repositories at [`TAG`]: one shallow
/// clone each, named by project. A folder there is always a whole checkout
/// ([`kicad_git::clone`] renames it into place only when git has finished).
fn clone_root(db: &Db) -> PathBuf {
    db.root().join("kicad-libraries").join(TAG)
}

/// Folders an administrator named instead, for a server that cannot run git
/// or reach GitLab: a KiCad install's `symbols`, `footprints` and `3dmodels`.
fn folders_file(db: &Db) -> PathBuf {
    db.root().join("kicad-libraries").join(format!("{TAG}-folders.json"))
}

/// The three folders an import reads, by repository.
#[derive(Clone)]
struct Source {
    dirs: [PathBuf; 3],
    from: &'static str,
}
impl Source {
    fn dir(&self, repo: Repo) -> &Path {
        &self.dirs[Repo::ALL.iter().position(|r| *r == repo).unwrap()]
    }
}

/// A clone running in the background, or the last one that failed.
#[derive(Default)]
struct Cloning {
    running: bool,
    line: String,
    error: Option<String>,
}

/// Per server data folder, so two stores in one process (the suite) never
/// share a clone's state.
static CLONES: std::sync::LazyLock<Mutex<HashMap<PathBuf, Cloning>>> = std::sync::LazyLock::new(Default::default);

/// The admin's folders when named, else the clones once all three are there.
fn ready(db: &Db) -> Result<Option<Source>, Error> {
    let file = folders_file(db);
    if file.is_file() {
        let saved: Value = serde_json::from_slice(&std::fs::read(&file).map_err(Error::internal)?).map_err(Error::internal)?;
        let dirs = Repo::ALL.map(|repo| PathBuf::from(saved[repo.install_folder()].as_str().unwrap_or_default()));
        if let Some(missing) = dirs.iter().find(|dir| !dir.is_dir()) {
            return Err(Error::bad_request(format!(
                "the KiCad folder {} named in setup no longer exists; name the folders again",
                missing.display()
            )));
        }
        return Ok(Some(Source { dirs, from: "folders" }));
    }
    let root = clone_root(db);
    let dirs = Repo::ALL.map(|repo| root.join(repo.project()));
    Ok(dirs.iter().all(|dir| dir.is_dir()).then_some(Source { dirs, from: "clone" }))
}

/// Start cloning whatever of the three is missing, unless a clone is already
/// running for this server. The clone runs on its own thread: the models are
/// gigabytes, far longer than any request may wait.
fn start_clone(db: &Db, base: &str) {
    let root = clone_root(db);
    {
        let mut clones = CLONES.lock().unwrap();
        let state = clones.entry(root.clone()).or_default();
        if state.running {
            return;
        }
        *state = Cloning { running: true, line: "starting git".into(), error: None };
    }
    let base = base.to_owned();
    std::thread::spawn(move || {
        let never = AtomicBool::new(false);
        let mut result = kicad_git::git_version().map(|_| ());
        for repo in Repo::ALL {
            if result.is_err() {
                break;
            }
            let mut report = |line: &str| {
                if let Some(state) = CLONES.lock().unwrap().get_mut(&root) {
                    state.line = format!("{}: {line}", repo.project());
                }
            };
            result = kicad_git::clone(&repo.url(&base), TAG, &root.join(repo.project()), &mut report, &never).map(|_| ());
        }
        let mut clones = CLONES.lock().unwrap();
        let state = clones.entry(root).or_default();
        state.running = false;
        state.error = result.err();
    });
}

/// Where the libraries stand: `ready` with every top-level symbol library,
/// `cloning` with git's latest progress line, or `failed` with why. A call
/// that finds neither the libraries nor a clone running starts one, so the
/// wizard polls this until it answers `ready`; a failure is reported once and
/// the next call tries again.
pub(super) fn libraries(db: &Db) -> Result<Value, Error> {
    libraries_from(db, clone_base().as_deref())
}

/// Where this server clones from: GitLab, or `BREP_PLM_KICAD_GIT_BASE` when
/// set. `off` there turns cloning off, so a server that has neither a clone
/// nor named folders answers `failed` at once; every test harness sets it, so
/// no suite starts a 5.6 GB download from GitLab.
fn clone_base() -> Option<String> {
    match std::env::var("BREP_PLM_KICAD_GIT_BASE") {
        Ok(base) if base.trim() == "off" => None,
        Ok(base) if !base.trim().is_empty() => Some(base.trim().to_owned()),
        _ => Some(kicad_git::BASE_URL.to_owned()),
    }
}

/// [`libraries`], cloning from `base` (a local repository in the suite), or
/// not at all when `base` is `None`.
fn libraries_from(db: &Db, base: Option<&str>) -> Result<Value, Error> {
    if let Some(source) = ready(db)? {
        let names = kicad_git::symbol_libraries(source.dir(Repo::Symbols)).map_err(Error::bad_request)?;
        let names: Vec<String> = names.into_iter().filter(|name| valid_library(name)).collect();
        if names.is_empty() {
            return Err(Error::bad_request("the KiCad symbols folder holds no symbol libraries"));
        }
        return Ok(json!({"status":"ready","release":TAG,"source":source.from,"libraries":names}));
    }
    let root = clone_root(db);
    {
        let mut clones = CLONES.lock().unwrap();
        if let Some(state) = clones.get_mut(&root) {
            if state.running {
                return Ok(json!({"status":"cloning","release":TAG,"line":state.line}));
            }
            if let Some(error) = state.error.take() {
                return Ok(json!({"status":"failed","release":TAG,"error":error,"folder":root}));
            }
        }
    }
    let Some(base) = base else {
        return Ok(json!({"status":"failed","release":TAG,"folder":root,
            "error":"this server is set not to clone KiCad's libraries (BREP_PLM_KICAD_GIT_BASE=off)"}));
    };
    start_clone(db, base);
    Ok(json!({"status":"cloning","release":TAG,"line":"starting git"}))
}

/// Name the server's own KiCad folders instead of cloning, or with every
/// field empty, forget them; the next [`libraries`] clones again.
pub(super) fn set_folders(db: &Db, folders: &FoldersRequest) -> Result<Value, Error> {
    let file = folders_file(db);
    let named = [&folders.symbols, &folders.footprints, &folders.models];
    if named.iter().all(|f| f.trim().is_empty()) {
        if file.is_file() {
            std::fs::remove_file(&file).map_err(Error::internal)?;
        }
        return Ok(json!({"status":"cleared","release":TAG}));
    }
    let mut saved = serde_json::Map::new();
    for (repo, folder) in Repo::ALL.into_iter().zip(named) {
        let path = PathBuf::from(folder.trim());
        if !path.is_absolute() || !path.is_dir() {
            return Err(Error::bad_request(format!(
                "the KiCad {} folder must be an existing folder on the server, given as a full path",
                repo.install_folder()
            )));
        }
        saved.insert(repo.install_folder().into(), json!(path));
    }
    std::fs::create_dir_all(file.parent().unwrap()).map_err(Error::internal)?;
    let temporary = file.with_extension(format!("{}.tmp", crate::auth::new_id()));
    std::fs::write(&temporary, serde_json::to_vec(&saved).map_err(Error::internal)?).map_err(Error::internal)?;
    std::fs::rename(&temporary, &file).map_err(Error::internal)?;
    libraries(db)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FoldersRequest {
    #[serde(default)]
    pub symbols: String,
    #[serde(default)]
    pub footprints: String,
    #[serde(default)]
    pub models: String,
}

/// Reclassify current imports without touching any document, revision or
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
        if modern { parts.push((part.id.clone(), library.to_string(), part.category.clone())); }
    }
    let mut changed = 0;
    for (id, library, old) in &parts {
        let category = category(db, root, library)?;
        if old != &category { db.update_part(id, &json!({"category":category}))?; changed += 1; }
    }
    Ok(json!({"parts":parts.len(),"reclassified":changed,"release":TAG}))
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

/// Reads from the three folders only; upstream model paths cannot escape them.
struct Files {
    source: Source,
    results: HashMap<String, Result<Option<String>, String>>,
}
impl Files {
    fn read(&mut self, repo: Repo, path: &str) -> Result<Option<String>, String> {
        safe_path(path)?;
        let key = format!("{}/{path}", repo.project());
        if let Some(result) = self.results.get(&key) {
            return result.clone();
        }
        let result = self.load(repo, path);
        self.results.insert(key, result.clone());
        result
    }
    fn load(&self, repo: Repo, path: &str) -> Result<Option<String>, String> {
        let file = self.source.dir(repo).join(path);
        if !file.is_file() {
            return Ok(None);
        }
        let length = std::fs::metadata(&file).map_err(|e| e.to_string())?.len();
        if length > LIMIT {
            return Err(format!("KiCad file exceeds 32 MiB: {path}"));
        }
        let bytes = std::fs::read(&file).map_err(|e| format!("{path}: {e}"))?;
        String::from_utf8(bytes).map(Some).map_err(|e| format!("{path}: {e}"))
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
        let unchanged = old["kicadImport"]["sourceHash"].as_str() == Some(hash.as_str());
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
    let source = ready(db)?.ok_or_else(|| {
        Error::bad_request("KiCad's libraries are not cloned yet; open the KiCad step of setup and wait for the clone")
    })?;
    let mut files = Files { source, results: HashMap::new() };
    let text = files
        .read(Repo::Symbols, &format!("{library}.kicad_sym"))
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
                .read(Repo::Footprints, &path)?
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
                                .read(Repo::Models, &path)?
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

