//! Native named roots. Configuration is host-owned, never taken from a request.
use super::{relative, Backend, Entry, Kind};
use std::{collections::BTreeMap, fs, io::{Read, Seek, SeekFrom, Write}, path::{Path, PathBuf}};

#[derive(Debug, Default)]
pub struct NativeFiles { roots: BTreeMap<String, PathBuf> }
impl NativeFiles {
    pub fn new(roots: impl IntoIterator<Item = (String, PathBuf)>) -> Result<Self, String> {
        let mut configured = BTreeMap::new();
        for (name, path) in roots {
            if name.is_empty() || name.len() > 64 || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-') {
                return Err("root names must contain 1 to 64 ASCII letters, digits, underscores or hyphens".into());
            }
            let path = path.canonicalize().map_err(err)?;
            if !path.is_dir() { return Err(format!("root {name} is not a directory")); }
            if configured.insert(name.clone(), path).is_some() { return Err(format!("duplicate root {name}")); }
        }
        Ok(Self { roots: configured })
    }
    /// Validate every component, including a symlink's canonical target. Return
    /// the lexical final path so deletion removes a file link, not its target.
    fn resolve(&self, root: &str, path: &str, create: bool) -> Result<PathBuf, String> {
        relative(path)?;
        let base = self.roots.get(root).ok_or_else(|| format!("unknown root {root}"))?;
        let parts: Vec<_> = path.split('/').filter(|p| !p.is_empty()).collect();
        let mut current = base.clone();
        // A configured root must still denote the directory it was bound to.
        if base.canonicalize().map_err(err)? != *base { return Err("configured root changed".into()); }
        for (i, part) in parts.iter().enumerate() {
            current.push(part);
            match fs::symlink_metadata(&current) {
                Ok(link) => {
                    let canonical = current.canonicalize().map_err(err)?;
                    if !canonical.starts_with(base) { return Err("path escapes named root after canonicalisation".into()); }
                    let meta = fs::metadata(&current).map_err(err)?;
                    if !meta.is_file() && !meta.is_dir() { return Err("only regular files and directories are supported; device files and sockets are refused".into()); }
                    if link.file_type().is_symlink() && meta.is_dir() {
                        return Err("directory symlinks are unsupported for file traversal".into());
                    }
                }
                Err(e) if create && i + 1 == parts.len() && e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(err(e)),
            }
        }
        Ok(current)
    }
}
fn err(e: impl std::fmt::Display) -> String { format!("files: {e}") }
fn entry(path: &Path) -> Result<Entry, String> {
    let meta = fs::metadata(path).map_err(err)?;
    let kind = if meta.is_file() { Kind::File } else if meta.is_dir() { Kind::Directory } else { return Err("only regular files and directories are supported".into()) };
    Ok(Entry {
        name: match path.file_name() { Some(name) => name.to_str().ok_or("file name is not UTF-8")?.to_string(), None => String::new() },
        kind,
        size: (kind == Kind::File).then_some(meta.len()),
        mtime: meta.modified().ok().and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok()).map(|d| d.as_secs_f64()),
    })
}
impl Backend for NativeFiles {
    fn stat(&self, root: &str, path: &str) -> Result<Entry, String> { entry(&self.resolve(root, path, false)?) }
    fn list(&self, root: &str, path: &str, max: usize) -> Result<Vec<Entry>, String> {
        let dir = self.resolve(root, path, false)?;
        let mut entries = Vec::new();
        for item in fs::read_dir(dir).map_err(err)? {
            if entries.len() >= max { return Err(format!("directory exceeds entry limit {max}")); }
            let item = item.map_err(err)?;
            let name = item.file_name().into_string().map_err(|_| "file name is not UTF-8")?;
            let child = super::join(path, &name)?;
            entries.push(self.stat(root, &child)?);
        }
        Ok(entries)
    }
    fn read(&self, root: &str, path: &str, offset: u64, length: usize) -> Result<Vec<u8>, String> {
        let path = self.resolve(root, path, false)?;
        if !fs::metadata(&path).map_err(err)?.is_file() { return Err("read requires a regular file".into()); }
        let mut file = fs::File::open(path).map_err(err)?;
        file.seek(SeekFrom::Start(offset)).map_err(err)?;
        let mut bytes = Vec::new();
        file.take(length as u64).read_to_end(&mut bytes).map_err(err)?;
        Ok(bytes)
    }
    fn write(&self, root: &str, path: &str, bytes: &[u8], overwrite: bool) -> Result<(), String> {
        let path = self.resolve(root, path, true)?;
        let mut options = fs::OpenOptions::new();
        options.write(true);
        if overwrite { options.create(true).truncate(true); } else { options.create_new(true); }
        let mut file = options.open(path).map_err(|e| if e.kind() == std::io::ErrorKind::AlreadyExists { "destination exists; pass overwrite: true".into() } else { err(e) })?;
        file.write_all(bytes).map_err(err)
    }
    fn mkdir(&self, root: &str, path: &str) -> Result<(), String> { fs::create_dir(self.resolve(root, path, true)?).map_err(err) }
    fn remove(&self, root: &str, path: &str, kind: Kind) -> Result<(), String> {
        let path = self.resolve(root, path, false)?;
        match kind { Kind::File => fs::remove_file(path), Kind::Directory => fs::remove_dir(path) }.map_err(err)
    }
    fn rename(&self, root: &str, path: &str, destination: &str, overwrite: bool) -> Result<(), String> {
        let source = self.resolve(root, path, false)?;
        let target = self.resolve(root, destination, true)?;
        if !overwrite && target.exists() { return Err("destination exists; pass overwrite: true".into()); }
        fs::rename(source, target).map_err(err)
    }
}
