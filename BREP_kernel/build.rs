//! Stamps the build with a hash of the kernel's own source.
//!
//! `BREP_KERNEL_SOURCE_HASH` is an FNV-1a 64 hash of source paths and their
//! complete contents, in sorted path order. Private builds exclude only whole
//! files proven test-only by the public exporter's reachability rules. Every
//! production file, including default-off feature modules, remains an input.
//! Test-only edits therefore preserve parts-library snapshot identities; edits
//! to production files (including their inline tests) still invalidate them.
//! Published builds hash every shipped `src/**/*.rs` file without extra tools.
//!
//! Deliberately not a git revision: the published crate has no `.git`, and a
//! HEAD-derived value would recompile the kernel on every docs-only commit.
//! Deliberately not a hand-bumped constant: a coupling nobody has to maintain
//! cannot be forgotten. If the source cannot be read the variable is simply not
//! set, and the consumer reads that as "no identity" rather than as a mismatch.

use std::fs;
use std::path::{Path, PathBuf};

fn collect(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            collect(&path, out)?;
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            out.push(path);
        }
    }
    Ok(())
}

fn source_files(root: &Path) -> std::io::Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    collect(&root.join("src"), &mut files)?;
    files.sort();
    // Standalone/public builds have no private classifier. Watch additions too.
    println!("cargo:rerun-if-changed=src");
    Ok(files)
}

fn source_hash(root: &Path, files: &[PathBuf]) -> std::io::Result<u64> {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for path in files {
        let relative = path.strip_prefix(root).unwrap_or(path);
        feed(relative.to_string_lossy().replace('\\', "/").as_bytes());
        feed(&[0]);
        feed(&fs::read(path)?);
        feed(&[0]);
    }
    Ok(hash)
}

fn main() {
    let root = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR"));
    println!("cargo:rerun-if-changed=build.rs");
    let result = source_files(&root).and_then(|files| {
        // Watching the directory would also watch all excluded test files.
        // A newly reachable module changes its watched parent or Cargo.toml.
        for path in &files {
            println!("cargo:rerun-if-changed={}", path.display());
        }
        source_hash(&root, &files)
    });
    match result {
        Ok(hash) => println!("cargo:rustc-env=BREP_KERNEL_SOURCE_HASH={hash:016x}"),
        Err(error) => println!("cargo:warning=BREP_KERNEL_SOURCE_HASH not set: {error}"),
    }
}

