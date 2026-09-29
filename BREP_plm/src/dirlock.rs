//! One process per data directory.
//!
//! The store holds the whole metadata in memory and writes each change to
//! `plm.sqlite` from there ([`crate::db`]). A second process opening the same
//! directory would load its own copy, and the two would overwrite each other's
//! changes row by row: SQLite keeps the FILE consistent, not two servers' ideas
//! of it. So [`Db::open`](crate::db::Db::open) takes an exclusive lock on
//! `<data>/plm.lock` first, and holds it for as long as the store is open.
//!
//! The lock is the operating system's advisory file lock on an open handle, so
//! it is released when the process exits however it exits — a crash leaves no
//! stale lock to clear by hand. The file itself stays, holding the pid of the
//! last process that took it, which is what the refusal names.

use std::fs::{File, OpenOptions, TryLockError};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

/// The lock file in a data directory.
pub const LOCK_FILE: &str = "plm.lock";

/// An exclusive hold on a data directory, released on drop.
#[derive(Debug)]
pub struct DirLock {
    path: PathBuf,
    // Held for its lock; never read after it is taken.
    _file: File,
}

impl DirLock {
    /// Take the directory `root`, or say which process has it.
    pub fn acquire(root: &Path) -> io::Result<DirLock> {
        std::fs::create_dir_all(root)?;
        let path = root.join(LOCK_FILE);
        let mut file = OpenOptions::new().read(true).write(true).create(true).truncate(false).open(&path)?;
        match file.try_lock() {
            Ok(()) => {}
            Err(TryLockError::WouldBlock) => {
                let mut text = String::new();
                let _ = file.read_to_string(&mut text);
                let holder = match text.trim() {
                    "" => "another process".to_string(),
                    pid => format!("another brep-plm process (pid {pid})"),
                };
                return Err(io::Error::new(
                    io::ErrorKind::ResourceBusy,
                    format!(
                        "{} is in use by {holder} — stop that server first, or give this one a different --data",
                        root.display()
                    ),
                ));
            }
            Err(TryLockError::Error(error)) => return Err(error),
        }
        file.set_len(0)?;
        file.seek(SeekFrom::Start(0))?;
        writeln!(file, "{}", std::process::id())?;
        file.flush()?;
        Ok(DirLock { path, _file: file })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

