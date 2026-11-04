// The browser mirror's adapter for the shared files engine. No explorer
// navigation, download, or model-key migration is involved in these operations.
use super::{Entry as StoredEntry, MirrorStore, DIR_PREFIX, FILE_PREFIX, PREFIX};
use crate::store::ModelStore;
use brep_mcp_core::files::{self, Backend, Entry, Kind};
use brep_mcp_core::files::{base64_decode, base64_encode};
use std::collections::BTreeMap;

#[derive(serde::Serialize, serde::Deserialize)]
struct Blob {
    base64: String,
    bytes: u64,
    mtime: Option<f64>,
}
fn now() -> Option<f64> {
    #[cfg(target_arch = "wasm32")]
    {
        Some(js_sys::Date::now() / 1000.0)
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .ok()
            .map(|d| d.as_secs_f64())
    }
}
fn model(path: &str) -> bool {
    crate::document_class::DocumentClass::of_name(path).is_some()
}
fn key(path: &str) -> String {
    if model(path) {
        MirrorStore::key(&format!("/models/{path}"))
    } else {
        format!("{FILE_PREFIX}{path}")
    }
}
fn name(path: &str) -> String {
    path.rsplit('/').next().unwrap_or("").into()
}
fn directory(path: &str) -> Entry {
    Entry {
        name: name(path),
        kind: Kind::Directory,
        size: None,
        mtime: None,
    }
}
impl MirrorStore {
    pub(super) fn file_settlement(&self, request: &files::Request) -> Option<Result<(), String>> {
        if self.pending.get() != 0 {
            return None;
        }
        let mut paths = vec![request.path.as_str()];
        match &request.operation {
            files::Operation::Copy { destination, .. } => paths = vec![destination],
            files::Operation::Move { destination, .. } => paths.push(destination),
            files::Operation::List { .. }
            | files::Operation::Stat
            | files::Operation::Read { .. } => return Some(Ok(())),
            _ => {}
        }
        for key in self.errors.unsynced.borrow().keys() {
            if let Some((path, _)) = Self::files_path(key) {
                if paths
                    .iter()
                    .any(|p| path == *p || path.starts_with(&format!("{p}/")))
                {
                    return Some(Err(format!("file persistence failed for {path}; the in-memory change is not confirmed saved")));
                }
            }
        }
        Some(Ok(()))
    }

    fn files_check(&self, root: &str, path: &str) -> Result<(), String> {
        if root != "documents" && root != "workspace" {
            return Err(format!(
                "unknown browser root {root}; available roots: documents, workspace"
            ));
        }
        if self.backend.plm_client().is_some() {
            return Err("files requires the local browser file store; PLM revisions are managed through the PLM API".into());
        }
        files::relative(path)?;
        if path.ends_with('/') || path.contains("//") {
            return Err("file paths must use single separators without a trailing slash".into());
        }
        Ok(())
    }
    fn files_value(&self, key: &str) -> Result<String, String> {
        let value = self.entries.borrow().get(key).cloned();
        match value {
            Some(StoredEntry::Resident(s)) => Ok(s),
            Some(StoredEntry::OnDisk { .. }) => {
                self.load(key);
                Err("file is loading; retry the operation after it loads".into())
            }
            None => Err("file does not exist".into()),
        }
    }
    fn files_blob(&self, path: &str) -> Result<Blob, String> {
        serde_json::from_str(&self.files_value(&key(path))?)
            .map_err(|_| "invalid stored file record".into())
    }
    fn files_parent(&self, root: &str, path: &str) -> Result<(), String> {
        let parent = path.rsplit_once('/').map(|(p, _)| p).unwrap_or("");
        if Backend::stat(self, root, parent)?.kind != Kind::Directory {
            return Err("parent must be a directory".into());
        }
        Ok(())
    }
    fn files_put(&self, path: &str, key: String, value: String) {
        self.stamp_local(&key);
        self.entries
            .borrow_mut()
            .insert(key.clone(), StoredEntry::Resident(value.clone()));
        let backend = self.backend.clone();
        let put_key = key.clone();
        self.push(path, &key, move || backend.put(&put_key, &value));
        self.mutations.set(self.mutations.get().wrapping_add(1));
    }
    fn files_delete(&self, path: &str, key: String) {
        self.stamp_local(&key);
        self.entries.borrow_mut().remove(&key);
        let backend = self.backend.clone();
        let delete_key = key.clone();
        self.push(path, &key, move || backend.delete(&delete_key));
        self.mutations.set(self.mutations.get().wrapping_add(1));
    }
    /// Interpret one existing persisted key as a file path. Reserved state is
    /// excluded, and legacy model keys keep their original meaning.
    fn files_path(key: &str) -> Option<(String, Kind)> {
        if let Some(path) = key.strip_prefix(PREFIX) {
            Some((crate::store::model_file_name(path), Kind::File))
        } else if let Some(path) = key.strip_prefix(FILE_PREFIX) {
            Some((path.into(), Kind::File))
        } else {
            key.strip_prefix(DIR_PREFIX)
                .map(|p| (p.trim_end_matches('/').into(), Kind::Directory))
        }
    }
}
impl Backend for MirrorStore {
    fn stat(&self, root: &str, path: &str) -> Result<Entry, String> {
        self.files_check(root, path)?;
        if path.is_empty() {
            return Ok(directory(path));
        }
        if self.entries.borrow().contains_key(&key(path)) {
            let (size, mtime) = if model(path) {
                let entries = self.entries.borrow();
                (
                    entries.get(&key(path)).ok_or("file does not exist")?.len(),
                    None,
                )
            } else {
                let blob = self.files_blob(path)?;
                (blob.bytes, blob.mtime)
            };
            return Ok(Entry {
                name: name(path),
                kind: Kind::File,
                size: Some(size),
                mtime,
            });
        }
        let prefix = format!("{path}/");
        if self
            .entries
            .borrow()
            .keys()
            .filter_map(|k| Self::files_path(k))
            .any(|(p, k)| (p == path && k == Kind::Directory) || p.starts_with(&prefix))
        {
            return Ok(directory(path));
        }
        Err("file or directory does not exist".into())
    }
    fn list(&self, root: &str, path: &str, max: usize) -> Result<Vec<Entry>, String> {
        if Backend::stat(self, root, path)?.kind != Kind::Directory {
            return Err("list requires a directory".into());
        }
        let prefix = if path.is_empty() {
            String::new()
        } else {
            format!("{path}/")
        };
        let mut children = BTreeMap::new();
        {
            let entries = self.entries.borrow();
            for (p, kind) in entries.keys().filter_map(|k| Self::files_path(k)) {
                let Some(rest) = p.strip_prefix(&prefix) else {
                    continue;
                };
                if rest.is_empty() {
                    continue;
                }
                let child = rest.split('/').next().unwrap();
                let is_dir = kind == Kind::Directory || rest.contains('/');
                children
                    .entry(child.to_string())
                    .and_modify(|d| *d |= is_dir)
                    .or_insert(is_dir);
                if children.len() > max {
                    return Err(format!("directory exceeds entry limit {max}"));
                }
            }
        }
        children
            .into_iter()
            .map(|(child, is_dir)| {
                let full = format!("{prefix}{child}");
                if is_dir {
                    Ok(directory(&full))
                } else {
                    Backend::stat(self, root, &full)
                }
            })
            .collect()
    }
    fn read(&self, root: &str, path: &str, offset: u64, length: usize) -> Result<Vec<u8>, String> {
        if Backend::stat(self, root, path)?.kind != Kind::File {
            return Err("read requires a file".into());
        }
        let bytes = if model(path) {
            self.files_value(&key(path))?.into_bytes()
        } else {
            let blob = self.files_blob(path)?;
            if blob.bytes > files::MAX_FILE {
                return Err(format!("file exceeds limit {} bytes", files::MAX_FILE));
            }
            let bytes = base64_decode(&blob.base64).map_err(|_| "invalid stored base64")?;
            if bytes.len() as u64 != blob.bytes {
                return Err("stored file size does not match its data".into());
            }
            bytes
        };
        let start = usize::try_from(offset)
            .unwrap_or(usize::MAX)
            .min(bytes.len());
        Ok(bytes[start..start.saturating_add(length).min(bytes.len())].to_vec())
    }
    fn write(&self, root: &str, path: &str, bytes: &[u8], overwrite: bool) -> Result<(), String> {
        self.files_check(root, path)?;
        self.files_parent(root, path)?;
        if self.entries.borrow().contains_key(&key(path)) && !overwrite {
            return Err("destination exists; pass overwrite: true".into());
        }
        if let Ok(entry) = Backend::stat(self, root, path) {
            if entry.kind != Kind::File {
                return Err("destination is a directory".into());
            }
            if !overwrite {
                return Err("destination exists; pass overwrite: true".into());
            }
        }
        if bytes.len() as u64 > files::MAX_FILE {
            return Err(format!("file exceeds limit {} bytes", files::MAX_FILE));
        }
        if model(path) {
            let text =
                std::str::from_utf8(bytes).map_err(|_| "model documents must be UTF-8 text")?;
            ModelStore::write(self, &format!("/models/{path}"), text)
        } else {
            let value = serde_json::to_string(&Blob {
                base64: base64_encode(bytes),
                bytes: bytes.len() as u64,
                mtime: now(),
            })
            .map_err(|e| e.to_string())?;
            self.files_put(path, key(path), value);
            Ok(())
        }
    }
    fn mkdir(&self, root: &str, path: &str) -> Result<(), String> {
        self.files_check(root, path)?;
        self.files_parent(root, path)?;
        if self.entries.borrow().contains_key(&key(path)) || Backend::stat(self, root, path).is_ok()
        {
            return Err("destination already exists".into());
        }
        self.files_put(path, format!("{DIR_PREFIX}{path}/"), String::new());
        Ok(())
    }
    fn remove(&self, root: &str, path: &str, kind: Kind) -> Result<(), String> {
        self.files_check(root, path)?;
        let entry = match Backend::stat(self, root, path) {
            Ok(entry) => entry,
            // Legacy stores infer folders from child keys. After their last
            // child was removed, there is no marker left to delete.
            Err(_)
                if kind == Kind::Directory && !self.entries.borrow().contains_key(&key(path)) =>
            {
                return Ok(())
            }
            Err(e) => return Err(e),
        };
        if entry.kind != kind {
            return Err("entry kind changed".into());
        }
        match kind {
            Kind::Directory => {
                if !Backend::list(self, root, path, files::MAX_ENTRIES)?.is_empty() {
                    return Err("directory is not empty; pass recursive: true".into());
                }
                self.files_delete(path, format!("{DIR_PREFIX}{path}/"));
            }
            Kind::File => self.files_delete(path, key(path)),
        }
        Ok(())
    }
    fn rename(
        &self,
        root: &str,
        path: &str,
        destination: &str,
        overwrite: bool,
    ) -> Result<(), String> {
        self.files_check(root, path)?;
        self.files_check(root, destination)?;
        self.files_parent(root, destination)?;
        if self.pending.get() != 0 {
            return Err("wait for pending storage writes before moving files".into());
        }
        let source = Backend::stat(self, root, path)?;
        if source.kind == Kind::File && key(path) == key(destination) {
            return Err("source and destination name the same file".into());
        }
        if let Ok(target) = Backend::stat(self, root, destination) {
            if !overwrite {
                return Err("destination exists; pass overwrite: true".into());
            }
            if target.kind != source.kind {
                return Err("source and destination kinds differ".into());
            }
            if target.kind == Kind::Directory
                && !Backend::list(self, root, destination, files::MAX_ENTRIES)?.is_empty()
            {
                return Err("destination directory must be empty".into());
            }
        }
        // Preflight the complete bounded move, including model encoding changes,
        // before changing any in-memory key. Persistence follows the normal
        // mirror write-behind contract and its existing error channel.
        let prefix = format!("{path}/");
        let mut moves = Vec::new();
        let mut bytes = 0u64;
        for (old, value) in self.entries.borrow().iter() {
            let Some((p, kind)) = Self::files_path(old) else {
                continue;
            };
            if p != path && !p.starts_with(&prefix) {
                continue;
            }
            if moves.len() >= files::MAX_ENTRIES {
                return Err(format!("tree exceeds entry limit {}", files::MAX_ENTRIES));
            }
            let new_path = format!("{destination}{}", &p[path.len()..]);
            files::relative(&new_path)?;
            let value = match value {
                StoredEntry::Resident(s) => s.clone(),
                StoredEntry::OnDisk { .. } => {
                    return Err("file is loading; load it before moving".into())
                }
            };
            let new_value = if kind == Kind::File && model(&p) != model(&new_path) {
                if model(&p) {
                    serde_json::to_string(&Blob {
                        base64: base64_encode(value.as_bytes()),
                        bytes: value.len() as u64,
                        mtime: now(),
                    })
                    .map_err(|e| e.to_string())?
                } else {
                    let blob: Blob =
                        serde_json::from_str(&value).map_err(|_| "invalid stored file record")?;
                    String::from_utf8(
                        base64_decode(&blob.base64).map_err(|_| "invalid stored base64")?,
                    )
                    .map_err(|_| "model documents must be UTF-8 text")?
                }
            } else {
                value
            };
            bytes = bytes.saturating_add(new_value.len() as u64);
            if bytes > files::MAX_TREE_BYTES {
                return Err(format!("tree exceeds byte limit {}", files::MAX_TREE_BYTES));
            }
            let new_key = if kind == Kind::Directory {
                format!("{DIR_PREFIX}{new_path}/")
            } else {
                key(&new_path)
            };
            moves.push((old.clone(), new_key, new_value));
        }
        let mut settlers = Vec::new();
        for (old, new, value) in &moves {
            for key in [old, new] {
                self.stamp_local(key);
                self.pending.set(self.pending.get() + 1);
                *self.in_flight.borrow_mut().entry(key.clone()).or_insert(0) += 1;
                settlers.push(self.settler(key));
            }
            let mut entries = self.entries.borrow_mut();
            entries.insert(new.clone(), StoredEntry::Resident(value.clone()));
            entries.remove(old);
        }
        self.mutations.set(self.mutations.get().wrapping_add(1));
        let request = self.backend.move_files(moves);
        let destination = destination.to_string();
        super::spawn(async move {
            let result = request.await;
            for settle in settlers {
                settle(&destination, result.clone());
            }
        });
        Ok(())
    }
}

