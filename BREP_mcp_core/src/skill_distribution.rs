//! Offline installation of reviewed Markdown skills. The trusted binary is
//! the provenance anchor; a package's self-declared publisher is not one.
use serde::{Deserialize, Serialize};
use std::{collections::BTreeSet, fs::{self, File, OpenOptions}, io::{Read, Write}, path::{Path, PathBuf}};

const MAX_BYTES: u64 = 1024 * 1024;
const FORMAT: &str = "brep-reviewed-skills-v1";
const PUBLISHER: &str = env!("CARGO_PKG_REPOSITORY");

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SkillFile { name: String, description: String, text: String, sha256: String }
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Bundle { format: String, package: String, publisher: String, version: String, skills: Vec<SkillFile> }
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Selection { active: String, previous: Option<String> }
#[derive(Debug, Serialize)]
pub struct Receipt { pub sha256: String, pub directory: PathBuf, pub previous: Option<String> }

pub fn sha256(bytes: &[u8]) -> String { crate::cad::canon::sha256_hex(bytes) }
/// Fetch the sole bundle reviewed into this binary. No network access.
pub fn bundle() -> Vec<u8> {
    let bundle = Bundle {
        format: FORMAT.into(), package: "BREP_mcp_core".into(), publisher: PUBLISHER.into(), version: env!("CARGO_PKG_VERSION").into(),
        skills: crate::skills::SKILLS.iter().map(|s| SkillFile {
            name:s.name.into(), description:s.description.into(), text:s.text.into(), sha256:sha256(s.text.as_bytes()),
        }).collect(),
    };
    serde_json::to_vec(&bundle).expect("embedded skill bundle serializes")
}
fn pin(pin: &str) -> Result<(), String> {
    if pin.len()!=64 || !pin.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c)) {
        return Err("expected a lowercase SHA256 pin from the reviewed artifact".into());
    }
    Ok(())
}
fn regular(path: &Path) -> Result<(), String> {
    let m=fs::symlink_metadata(path).map_err(|e| e.to_string())?;
    if !m.is_file() || m.file_type().is_symlink() { return Err("managed file must be regular, not a symlink".into()); }
    #[cfg(unix)] {
        use std::os::unix::fs::PermissionsExt;
        if m.permissions().mode() & 0o111 != 0 { return Err("skill data files must not be executable".into()); }
    }
    Ok(())
}
/// Bounded reads; no archive extraction or caller-controlled resource paths.
pub fn read_bundle(path: &Path) -> Result<Vec<u8>, String> {
    regular(path)?;
    let mut bytes=Vec::new();
    File::open(path).map_err(|e| e.to_string())?.take(MAX_BYTES+1).read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    if bytes.len() as u64>MAX_BYTES { return Err("skill bundle exceeds 1 MiB".into()); }
    Ok(bytes)
}
fn parse(bytes: &[u8], expected: &str) -> Result<Bundle, String> {
    pin(expected)?;
    if bytes.len() as u64>MAX_BYTES || sha256(bytes)!=expected { return Err("skill bundle size or SHA256 mismatch".into()); }
    let bundle:Bundle=serde_json::from_slice(bytes).map_err(|e| format!("invalid skill bundle: {e}"))?;
    if bundle.format!=FORMAT || bundle.package!="BREP_mcp_core" || bundle.publisher!=PUBLISHER || bundle.version.is_empty() || bundle.version.len()>128 {
        return Err("unsupported skill bundle provenance or format".into());
    }
    let allow:BTreeSet<_>=crate::skills::SKILLS.iter().map(|s| s.name).collect();
    let names:BTreeSet<_>=bundle.skills.iter().map(|s| s.name.as_str()).collect();
    if names!=allow || bundle.skills.len()!=allow.len() { return Err("skill catalogue differs from the reviewed allowlist".into()); }
    for skill in &bundle.skills {
        let header=format!("---\nname: {}\ndescription: {}\n---\n",skill.name,skill.description);
        if (skill.description.contains('\r') || skill.description.contains('\n')) || !skill.text.starts_with(&header) || sha256(skill.text.as_bytes())!=skill.sha256 {
            return Err("skill frontmatter or content digest mismatch".into());
        }
    }
    Ok(bundle)
}
fn new_directory(path: &Path) -> Result<(), String> {
    let mut builder=fs::DirBuilder::new();
    #[cfg(unix)] { use std::os::unix::fs::DirBuilderExt; builder.mode(0o700); }
    builder.create(path).map_err(|e| e.to_string())
}
fn directory(path: &Path) -> Result<(), String> {
    let m=fs::symlink_metadata(path).map_err(|e| e.to_string())?;
    if !m.is_dir() || m.file_type().is_symlink() { return Err("managed directory must not be a symlink".into()); }
    Ok(())
}
fn names(path: &Path) -> Result<BTreeSet<String>, String> {
    fs::read_dir(path).map_err(|e| e.to_string())?.map(|e| {
        e.map_err(|e| e.to_string())?.file_name().into_string().map_err(|_| "non-UTF8 managed entry".into())
    }).collect()
}
fn write_new(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let mut options=OpenOptions::new(); options.write(true).create_new(true);
    #[cfg(unix)] { use std::os::unix::fs::OpenOptionsExt; options.mode(0o600); }
    let mut file=options.open(path).map_err(|e| e.to_string())?;
    file.write_all(bytes).and_then(|_| file.sync_all()).map_err(|e| e.to_string())
}
/// Export refuses replacement. The caller chooses the destination explicitly.
pub fn export(path: &Path) -> Result<String, String> {
    let bytes=bundle(); write_new(path,&bytes)?; Ok(sha256(&bytes))
}
fn sync_directory(path: &Path) -> Result<(), String> {
    #[cfg(unix)] { File::open(path).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?; }
    let _=path; Ok(())
}
fn open_root(root: &Path) -> Result<File, String> {
    if !root.exists() { new_directory(root)?; }
    directory(root)?;
    let allowed:BTreeSet<_>=[".lock","versions","selection.json","selection.next"].into_iter().map(str::to_owned).collect();
    if !names(root)?.is_subset(&allowed) { return Err("destination contains unmanaged entries".into()); }
    crate::host::private_dir(root)?;
    let path=root.join(".lock");
    if fs::symlink_metadata(&path).is_ok() { regular(&path)?; }
    let mut options=OpenOptions::new();options.read(true).write(true).create(true).truncate(false);
    #[cfg(unix)] { use std::os::unix::fs::OpenOptionsExt; options.mode(0o600); }
    let lock=options.open(path).map_err(|e| e.to_string())?;
    lock.try_lock().map_err(|_| "skill destination is already in use".to_string())?;
    let versions=root.join("versions");
    if !versions.exists() { new_directory(&versions)?; }
    directory(&versions)?;
    Ok(lock)
}
fn selection(root: &Path) -> Result<Option<Selection>, String> {
    let path=root.join("selection.json");
    match fs::symlink_metadata(&path) {
        Err(e) if e.kind()==std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e.to_string()),
        Ok(_) => {}
    }
    let selection:Selection=serde_json::from_slice(&read_bundle(&path)?).map_err(|e| e.to_string())?;
    pin(&selection.active)?;
    if let Some(previous)=&selection.previous { pin(previous)?; }
    Ok(Some(selection))
}
fn verify_version(root: &Path, digest: &str) -> Result<PathBuf, String> {
    pin(digest)?;
    let path=root.join("versions").join(digest); directory(&path)?;
    let bytes=read_bundle(&path.join("bundle.json"))?;
    let bundle=parse(&bytes,digest)?;
    let mut expected:BTreeSet<_>=bundle.skills.iter().map(|s| s.name.clone()).collect(); expected.insert("bundle.json".into());
    if names(&path)?!=expected { return Err("unexpected file or asset in installed skill bundle".into()); }
    for skill in bundle.skills {
        let directory_path=path.join(skill.name);directory(&directory_path)?;
        if names(&directory_path)?!=BTreeSet::from(["SKILL.md".into()]) { return Err("unexpected installed skill asset".into()); }
        if read_bundle(&directory_path.join("SKILL.md"))?!=skill.text.as_bytes() { return Err("installed skill content mismatch".into()); }
        #[cfg(unix)] {
            use std::os::unix::fs::PermissionsExt;
            if fs::metadata(directory_path.join("SKILL.md")).map_err(|e| e.to_string())?.permissions().mode() & 0o111 !=0 {
                return Err("installed Markdown must not be executable".into());
            }
        }
    }
    Ok(path)
}
fn select(root: &Path, selection: &Selection) -> Result<(), String> {
    let path=root.join("selection.json");
    if fs::symlink_metadata(&path).is_ok() { regular(&path)?; }
    let temporary=root.join("selection.next");
    if fs::symlink_metadata(&temporary).is_ok() { regular(&temporary)?; fs::remove_file(&temporary).map_err(|e| e.to_string())?; }
    write_new(&temporary,&serde_json::to_vec(selection).map_err(|e| e.to_string())?)?;
    if let Err(e)=fs::rename(&temporary,&path) { let _=fs::remove_file(temporary); return Err(e.to_string()); }
    sync_directory(root).map_err(|e| format!("selection was replaced but durability sync failed; inspect before retry: {e}"))
}
fn receipt(root: &Path, selection: Selection) -> Result<Receipt, String> {
    let path=verify_version(root,&selection.active)?;
    Ok(Receipt {sha256:selection.active,directory:path.canonicalize().map_err(|e| e.to_string())?,previous:selection.previous})
}
pub fn inspect(root: &Path) -> Result<Receipt, String> {
    directory(root)?;
    let _lock=open_root(root)?;
    receipt(root,selection(root)?.ok_or("no installed skill selection")?)
}
pub fn install(bytes: &[u8], expected: &str, root: &Path) -> Result<Receipt, String> {
    install_reviewed(bytes,expected,root,&bundle())
}
// The only production caller passes this binary's embedded reviewed bytes.
fn install_reviewed(bytes: &[u8], expected: &str, root: &Path, reviewed: &[u8]) -> Result<Receipt, String> {
    let bundle=parse(bytes,expected)?;
    if bytes!=reviewed { return Err("bundle is not the reviewed snapshot embedded in this binary".into()); }
    let _lock=open_root(root)?;
    let current=selection(root)?;
    if let Some(current)=&current { verify_version(root,&current.active)?; }
    if current.as_ref().is_some_and(|s| s.active==expected) { return receipt(root,current.unwrap()); }
    let destination=root.join("versions").join(expected);
    if fs::symlink_metadata(&destination).is_ok() { verify_version(root,expected)?; }
    else {
        let staging=root.join("versions").join(format!(".staging-{}",uuid::Uuid::new_v4()));
        new_directory(&staging)?;
        let staged=(|| {
            write_new(&staging.join("bundle.json"),bytes)?;
            for skill in bundle.skills {
                let path=staging.join(skill.name); new_directory(&path)?;
                write_new(&path.join("SKILL.md"),skill.text.as_bytes())?; sync_directory(&path)?;
            }
            sync_directory(&staging)?;
            fs::rename(&staging,&destination).map_err(|e| e.to_string())?;
            sync_directory(&root.join("versions"))
        })();
        if staged.is_err() { let _=fs::remove_dir_all(staging); }
        staged?;
    }
    let next=Selection {active:expected.into(),previous:current.map(|s| s.active)};
    verify_version(root,expected)?;
    select(root,&next)?;
    receipt(root,next)
}
pub fn rollback(root: &Path, expected: &str) -> Result<Receipt, String> {
    pin(expected)?;
    directory(root)?;
    let _lock=open_root(root)?;
    let current=selection(root)?.ok_or("no installed skill selection")?;
    if current.previous.as_deref()!=Some(expected) { return Err("rollback pin does not identify the previous reviewed version".into()); }
    verify_version(root,&current.active)?; verify_version(root,expected)?;
    let next=Selection {active:expected.into(),previous:Some(current.active)};
    select(root,&next)?; receipt(root,next)
}

